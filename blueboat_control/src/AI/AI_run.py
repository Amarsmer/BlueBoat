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
from std_msgs.msg import String, Float32, Float32MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker

# Custom libraries
from blueboat_control import ROV
import custom_functions as cf
from blueboat_interfaces.srv import RequestPath

# Training specific custom librairies
from blueboat_control import ROV
from backprop import NN
from online_training import PyTorchOnlineTrainer

# Training specific librairies
import torch
import json
import threading
import atexit

class Controller(Node):
    def __init__(self):

        super().__init__('ai_control', namespace='blueboat')

        # self.declare_parameter('name', 'data')
        self.declare_parameter('weight_name', '')        # Will load a saved weight file if specified, otherwise will create some
        self.declare_parameter('train', True)            # Wether weight are updated, 'False' implies testing
        self.declare_parameter('automate', True)         # Test automation will change the robot's pose if some loss requirements are met
        self.input_string = ['','']                      # Meant to be used as ['instruction', 'argument']

        self.rov = ROV(self, thrust_visual = True)

        ################## ROS2 Communication ##################

        self.odom_subscriber = self.create_subscription(Odometry, '/blueboat/odom', self.odom_callback, 10)
        self.str_input_subscriber = self.create_subscription(String, '/blueboat/input_str', self.str_input_callback, 10) # Used to interact with the training process
        self.pose_arrow_publisher = self.create_publisher(Marker, "/pose_arrow", 10)

        self.data_publisher = self.create_publisher(Float32MultiArray, "/monitoring_data", 10)
        self.network_publisher = self.create_publisher(Float32MultiArray, "/network_data", 10)
        
        self.dt = 0.01 # Used both for run and pose computation
        self.timer = self.create_timer(self.dt, self.run)

        self.future = None # Used for client requests

        # Create a client for path request
        self.client = self.create_client(RequestPath, '/path_request')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")

        """
        ################# Reference weighting matrices meant to be equivalent to MPC

        # Weighting matrices
        self.Q_weight = np.diag([50, # x
                                 50, # y 
                                 40, # psi
                                 1, # u
                                 1, # v
                                 1  # r
                                 ])
        
        self.R_weight = np.diag([0.015, # u1
                                 0.015  # u2
                                 ])
        """
        
        ################## Initiating variables ##################

        self.ai_path = Path()
        
        # Weighting matrices
        self.Q_weight = np.diag([50, # x
                                 50, # y 
                                 10, # psi
                                 1, # u
                                 1, # v
                                 1  # r
                                 ])
        
        self.R_weight = np.diag([1e-5, # u1
                                 1e-5  # u2
                                 ])
        # TODO The difference of R weight between AI and MPC may come from delta_t applied to the gradient, further investigation required

        ### Create pytorch network

        # Network parameters
        self.HL_size = 40
        input_size = 6 # x, y, psi, u, v, r
        output_size = 2 # u1, u2
        self.learning_rate = 1e-4

        self.trainer = None
        self.training_initiated = False

        # Weight loading
        weight_name = self.get_parameter('weight_name').get_parameter_value().string_value
        if weight_name == '':
            self.get_logger().info(f"No weight loaded. Initializing random weights with hidden layer size: {self.HL_size}.")
            self.network = NN(input_size, self.HL_size, output_size)

        else:
            # Check if the file exists
            try:
                with open(f'/home/noe/ros2_ws/saved_weights/{weight_name}.json') as fp:
                    json_obj = json.load(fp)
                
            # If it does not, display error message and create a new network
            except:
                self.get_logger().info(f"#################### ERROR: no weight file with the name: {weight_name}. Initializing random weights with hidden layer size: {self.HL_size}. ####################")
                self.network = NN(input_size, self.HL_size, output_size)

            # If it exists, adjust the hidden layer size and load the network
            else:
                self.HL_size = len(json_obj["input_weights"][0][:])
                self.network = NN(input_size, self.HL_size, output_size)
                self.network.load_weights_from_json(json_obj)
                self.get_logger().info(f"Loading weight json: {weight_name}.")

        ### Test automation
        self.automate = self.get_parameter('automate').get_parameter_value().bool_value
        if self.automate:
            self.loss_threshold = 15.
            self.previous_loss = 1e10 # Initialized at an arbitrarily high value instead of None to reduce the number of if statements
            self.minimal_loss_timer = None
            self.acceptable_loss_delay = 10. # If the loss is below the threshold for this amount of time (s), the robot is moved
            self.pose_index = 0
            self.initial_poses = [np.array([4., 4., 0., 0., 0., 1.]),
                                np.array([-4., -4., 0., 0., 0., 4.]),
                                np.array([4., -4., 0., 0., 0., 1.]),
                                np.array([-4., 4., 0., 0., 0., np.pi/2]),
                                np.array([0., -4., 0., 0., 0., np.pi/2.]),
                                np.array([-4., 4., 0., 0., 0., -np.pi/2]),
                                np.array([0., -4., 0., 0., 0., -np.pi/2.]),
                                np.array([15., 4., 0., 0., 0., 1.]),
                                np.array([-4., 15., 0., 0., 0., 4.]),
                                np.array([15., 15., 0., 0., 0., 0.])]

        self.current_time = self.get_time()

        # Initiate monitoring data, both stored as .npy and published on a topic
        self.monitoring = []
        self.monitoring.append(['x','y','psi','x_d','y_d','psi_d','u1','u2', 'grad1', 'grad2', 'loss_X', 'loss_u', 'skew', 't']) # Naming the variables in the first row, useful as is and even more so if data points change between version

        self.date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')

    def get_time(self):
        s,ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns*1e-9

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
    
    def str_input_callback(self, msg: String):
        self.input_string = msg.data.split()

    def updateRobotState(self):
        # Extract position and speed as column vectors
        position = np.array([self.rov.current_pose[x] for x in [0, 1, 5]]).reshape(-1, 1)
        speed = np.array([self.rov.current_twist[x] for x in [0, 1, 5]]).reshape(-1, 1)

        # Concatenate vertically (stack columns)
        self.state = np.vstack([position, speed])

        self.trainer.updateState(self.state)

    def run(self):
        ################## Wait for the robot model to be initialized ##################

        if not self.rov.ready():
            return

        # Update time
        self.current_time = self.get_time()

        ################## Initialize ##################

        if not self.training_initiated: # This code used to be in a while loop and requires adjustements to work as a ROS2 node
            self.training_initiated = True

            self.t0 = self.get_time() # Initial time for data collection
                    
            # Initialize trainer
            self.trainer = PyTorchOnlineTrainer(self.rov, self.network, self.learning_rate, self.Q_weight, self.R_weight)

            train = self.get_parameter('train').get_parameter_value().bool_value #Boolean

            # Main training
            self.target = [0.,0.,0.,0.,0.,0.] # Initial target

            self.updateRobotState() # The trainer thread requires manual update of the robot's pose and twist
            self.trainer.updateTarget(self.target) # To be used for trajectory tracking

            self.get_logger().info(f"\n Starting training session")

            self.training_thread = threading.Thread(target=self.trainer.train, args=(self.target,)) # Start training process on a separate thread
            self.trainer.running = True
            self.training_thread.start()

        ################## Request Path ##################

        # Check if previous future is still pending
        if self.future is not None:
            if self.future.done():
                try:
                    result = self.future.result()
                    if result is not None:
                        self.ai_path = result.path
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

        if self.ai_path.poses: # Make sure the path is not empty

            self.target = self.compute_target(self.ai_path)
            self.trainer.updateTarget(self.target)

            # Display target in gazebo
            target_pose = cf.make_pose(self.target)
            cf.create_pose_marker(target_pose, self.pose_arrow_publisher)

        self.updateRobotState()

        ################## Training automation ##################

        if self.trainer.loss and self.automate: # Make sure the loss has been initialized

            # Detect when loss is below threshold (edge detection)
            if self.trainer.loss < self.loss_threshold :
                if self.previous_loss >= self.loss_threshold :
                    self.minimal_loss_timer = self.current_time
            else:
                self.minimal_loss_timer = None


            # Change robot's pose after loss remains under threshold for a set time
            if self.minimal_loss_timer and (self.current_time - self.minimal_loss_timer) > self.acceptable_loss_delay:
                    cf.set_pose_gz(self.initial_poses[self.pose_index])
                    self.pose_index += 1
                    self.pose_index %= len(self.initial_poses) # Makes sure the index wraps around instead of getting outside of the list
                    self.minimal_loss_timer = None
                    self.get_logger().info(f"\n Loss requirements met. Current pose index: {self.pose_index}")

            self.previous_loss = self.trainer.loss

        ################## Save and publish data for monitoring ##################

        if self.trainer.trainer_set: # Make sure the training has started
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
        
        if self.input_string[0] == 'stop': # Stop training session from terminal
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
node = Controller()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()