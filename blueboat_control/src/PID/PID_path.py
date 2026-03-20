#!/usr/bin/env python3

# rclpy
from rclpy.node import Node, QoSProfile
from rclpy.qos import QoSDurabilityPolicy
import rclpy

# Common python libraries
import time
import numpy as np
from datetime import datetime
import os

# ROS2 msg libraries
from std_msgs.msg import String, Float32, Float64, Float32MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker

# Custom libraries
from blueboat_control import ROV
import custom_functions as cf
from blueboat_interfaces.srv import RequestPath

# Training specific librairies
import torch
import json
import threading
import atexit

class PathController(Node):
    def __init__(self):

        super().__init__('PID_path_control', namespace='blueboat')
        
        self.rov = ROV(self, thrust_visual = True, PID=True)
        self.command = [0.,0.,0.]

        ################## ROS2 Communication ##################

        # Subscribe to PID cmd to display the thrust control in rviz. I am a aware this is heinously ugly and spaghet (reader, this will not improve)
        self.cmd1_subscriber = self.create_subscription(Float64, '/blueboat/cmd_thruster1', self.thrust1, 10)
        self.cmd2_subscriber = self.create_subscription(Float64, '/blueboat/cmd_thruster2', self.thrust2, 10)
        try:
            self.cmd3_subscriber = self.create_subscription(Float64, '/blueboat/cmd_thruster3', self.thrust3, 10)
        except:
            pass


        self.odom_subscriber = self.create_subscription(Odometry, '/blueboat/odom', self.odom_callback, 10)
        self.pose_arrow_publisher = self.create_publisher(Marker, '/pose_arrow', 10)
        self.cmd_pose_publisher = self.create_publisher(PoseStamped, "/blueboat/cmd_pose", 10)

        self.data_publisher = self.create_publisher(Float32MultiArray, "/monitoring_data", 10)
        self.network_publisher = self.create_publisher(Float32MultiArray, "/network_data", 10)
        
        self.dt = 0.01 # Used both for run and pose computation
        self.timer = self.create_timer(self.dt, self.run)

        self.future = None # Used for client requests

        # Create a client for path request
        self.client = self.create_client(RequestPath, '/path_request')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")


        self.path = Path()

        self.current_time = self.get_time()

        # Initiate monitoring data, both stored as .npy and published on a topic
        self.monitoring = []
        self.monitoring.append(['t','x','y','psi','x_d','y_d','psi_d','u1','u2','u3']) # Naming the variables in the first row, useful as is and even more so if data points change between version

        self.date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')

    def get_time(self):
        s,ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns*1e-9


    def thrust1(self, msg: Float64):
        self.command[0] = msg.data

    def thrust2(self, msg: Float64):
        self.command[1] = msg.data

    def thrust3(self, msg: Float64):
        self.command[2] = msg.data

    def odom_callback(self, msg: Odometry):
        pose, twist = cf.odometry(msg)

        self.rov.current_pose = pose
        self.rov.current_twist = twist

    def compute_target(self, path):

        def get_yaw_from_quaternion(q):
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return np.arctan2(siny_cosp, cosy_cosp)

        # Extract current target and next step target
        poses = path.poses[:2]
        present = poses[0].pose
        future = poses[1].pose

        # Compute position
        x = future.position.x
        y = future.position.y
        psi = get_yaw_from_quaternion(future.orientation)
        psi = (psi + np.pi) % (2 * np.pi) - np.pi

        # Compute speeds
        dx = x - present.position.x
        dy = y - present.position.y
        u = np.hypot(dx, dy) / self.dt

        psi_prev = get_yaw_from_quaternion(present.orientation)

        psi_mid = (psi + psi_prev) / 2.0
        dx_b =  np.cos(psi_mid) * dx + np.sin(psi_mid) * dy
        dy_b = -np.sin(psi_mid) * dx + np.cos(psi_mid) * dy
        v = dy_b / self.dt 

        dpsi = (psi - psi_prev + np.pi) % (2 * np.pi) - np.pi
        r = dpsi / self.dt
        
        return [x, y, psi, u, v, r]

    def updateRobotState(self):
        # Extract position and speed as column vectors
        position = np.array([self.rov.current_pose[x] for x in [0, 1, 5]]).reshape(-1, 1)
        speed = np.array([self.rov.current_twist[x] for x in [0, 1, 5]]).reshape(-1, 1)

        # Concatenate vertically (stack columns)
        self.state = np.vstack([position, speed])

    def run(self):
        ################## Wait for the robot model to be initialized ##################

        if not self.rov.ready():
            return

        # Update time
        self.current_time = self.get_time()

        ################## Initialize ##################
        self.t0 = self.get_time() # Initial time for data collection
                
        self.updateRobotState()

        ################## Request Path ##################

        # Check if previous future is still pending
        if self.future is not None:
            if self.future.done():
                try:
                    result = self.future.result()
                    if result is not None:
                        self.path = result.path
                        # self.get_logger().info(f"Received path with {len(self.mpc_path.poses)} poses.")
                    else:
                        self.get_logger().error("Service returned None.")
                except Exception as e:
                    self.get_logger().error(f"Service call raised exception: {e}")
                finally:
                    self.future = None
                return

        # Send new request
        request = RequestPath.Request()
        request.path_request.data = np.linspace(self.current_time, self.current_time + self.dt, 2, dtype=float)

        self.future = self.client.call_async(request)

        if self.path.poses: # Make sure the path is not empty
            # Publish to cmd_pose
            cmd_pose = self.path.poses[0]
            out = PoseStamped()
            out.header = cmd_pose.header
            out.pose = cmd_pose.pose
            self.cmd_pose_publisher.publish(out)

            # self.target = self.compute_target(self.path)

            # Display target in gazebo
            target_pose = cf.make_pose(self.compute_target(self.path))
            cf.create_pose_marker(target_pose, self.pose_arrow_publisher)

        self.updateRobotState()
        self.rov.move(self.command)

        ################## Save and publish data for monitoring ##################

        if False:
            x_m = self.trainer.state[0]
            y_m = self.trainer.state[1]
            psi_m = self.trainer.state[5]

            x_d_m = self.trainer.target[0]
            y_d_m = self.trainer.target[1]
            psi_d_m = self.trainer.target[2]

            u = self.trainer.u.ravel()
            grad = self.trainer.gradient_display.ravel()
            loss = self.trainer.loss_display.ravel()
            skew = self.trainer.skew
            t = self.get_time() - self.t0

            data_array = [x_m, y_m, psi_m, x_d_m, y_d_m , psi_d_m, u[0],u[1], grad[0], grad[1], loss[0], loss[1], skew, t]

            self.monitoring.append(data_array)

            publisher_msg = Float32MultiArray()
            publisher_msg.data = data_array
            self.data_publisher.publish(publisher_msg)

            publisher_msg = Float32MultiArray()
            publisher_msg.data = self.trainer.error_display
            self.network_publisher.publish(publisher_msg)
            
            # Debug info
            # self.get_logger().info(f"Grad: {self.trainer.gradient_display}") 
            # self.get_logger().info(f"U: {self.trainer.u}") 
            # self.get_logger().info(f"Delta_t: {self.trainer.delta_t_display} \n") 
            # self.get_logger().info(f"\n Internal state: {self.trainer.state}")
            # self.get_logger().info(f"\n Internal error: {self.trainer.error}") 
            # self.get_logger().info(f"\n Train state: {self.trainer.state_train_display}")
            # self.get_logger().info(f"\n Train target: {self.trainer.target}") 
            # self.get_logger().info(f"\n Train error: {self.trainer.error_display[2]}")
            # self.get_logger().info(f"\n Network input: {self.trainer.input_display}")
            # self.get_logger().info(f"\n Network input: {self.trainer.skew}")
            
        ################## Stop training and record data ##################
        
        if False: # Stop training session from terminal
            weight_name = self.input_string[1]
            self.input_string = ['','']
            self.trainer.running = False
            self.training_thread.join(timeout=5)

            # Save the weights
            json_obj = self.network.save_weights_to_json()
            with open(f'saved_weights/{weight_name}.json', 'w') as fp:
                
                json.dump(json_obj, fp)

            self.get_logger().info("Training stopped")

            title = f'data/AI_data/{self.date}-weightfile_{weight_name}-HL_{self.HL_size}-AI_data'
            np.save(title, self.monitoring)


rclpy.init()
node = PathController()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()