#!/usr/bin/env python3

# rclpy
from rclpy.node import Node, QoSProfile
from rclpy.qos import QoSDurabilityPolicy
import rclpy

# Common python libraries
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from datetime import datetime

# ROS2 msg libraries
from std_msgs.msg import String, Bool, Float32, Float32MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker

# Custom libraries
from urdf_parser_py import urdf
import ur_mpc
import PID
from blueboat_control import ROV
from blueboat_interfaces.srv import RequestPath
import custom_functions as cf

class Controller(Node):
    def __init__(self):

        super().__init__('blueboat_control', namespace='blueboat')

        self.declare_parameter('controller_type', 'MPC') 
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value

        self.declare_parameter('simulation', True) 
        self.isSimulation = self.get_parameter('simulation').get_parameter_value().bool_value

        self.declare_parameter('use_pinger', False) 
        self.use_pinger = self.get_parameter('use_pinger').get_parameter_value().bool_value

        self.odom_subscriber = self.create_subscription(Odometry, '/blueboat/odom', self.odom_callback, 10)
        self.pinger_subscriber = self.create_subscription(Float32MultiArray, '/blueboat/pinger_coordinates', self.pinger_callback, 10)
        self.ready_subscriber = self.create_subscription(Bool, '/blueboat/controller_ready', self.ready_callback, 10)

        self.data_publisher = self.create_publisher(Float32MultiArray, "/monitoring_data", 10)
        self.target_publisher = self.create_publisher(Float32MultiArray,'/controller_target', 10)
        self.thruster_input_publisher = self.create_publisher(Float32MultiArray, "/thruster_input", 10)
        self.pose_arrow_publisher = self.create_publisher(Marker, "/pose_arrow", 10)

        # Create a client for path request
        self.client = self.create_client(RequestPath, '/path_request')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")
        
        self.future = None # Used for client requests

        self.time_set = False
        self.initial_time = None
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.current_pose = None
        self.current_twist = None

        self.ready = False
        self.init = False

        # Initialize controller 
        self.controller_path = Path()

        # MPC Parameters
        if self.controller_type == 'MPC':
            self.mpc_horizon = 15
            self.mpc_time = 2.5
            max_linear_bound = 20.0
            min_linear_bound = -20.0
            self.input_bounds = {"lower": np.array([min_linear_bound, min_linear_bound]),
                                "upper": np.array([max_linear_bound, max_linear_bound]),
                                "idx":   np.array([0, 1])
                                }

            self.Q_weight = np.diag([50, # x
                                    50, # y 
                                    30, # psi
                                    1, # u
                                    1, # v
                                    1  # r
                                    ])
            
            self.R_weight = np.diag([0.015, # u1
                                    0.015  # u2
                                    ])

            # Initialize MPC solver
            self.controller = None # Updated at the start of spin

            self.path_time = self.mpc_time
            self.path_steps = self.mpc_horizon

        # PID Parameters
        if self.controller_type == 'PID':
            self.path_time = self.dt
            self.path_steps = 2
            
            self.outer_gains = {'x': (25., 5., 0.),
                                'psi': (15., 2., 0.)}

            self.inner_gains = {'u': (1., 0., 0.),
                                'r': (1., 0., 0.)}
            
            self.thruster_limits = {"min": np.array([-20.0, -20.0]),   
                                    "max": np.array([ 20.0,  20.0])}

            radius = 0.59/2
            self.B_matrix = B = np.array([[1.        ,1.],
                                          [0.        ,0.],
                                          [radius,-radius]])

        # LoS Parameters
        if self.controller_type ==  'LoS':
            self.path_time = self.dt
            self.path_steps = 2

            self.k_v = 0.03
            self.k_psi = 20.0

            self.pinger_target = None
            self.stopping_sequence = False # Used as a safety to stop LoS control when it gets close to target
            self.stopping_time = None

        # Initialize monitoring values
        self.monitoring = []
        self.monitoring.append(['t','x','y','psi','x_d','y_d','psi_d','u1','u2'])

        self.t_record = self.get_time()

        ctrl = self.controller_type
        date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')
        sim = 'simulation' if self.isSimulation else 'real'
        self.title = f'data/{ctrl}_data/{date}-{ctrl}_{sim}_data'

    def get_time(self):
        s,ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns*1e-9

    def odom_callback(self, msg: Odometry):
        pose, twist = cf.odometry(msg)

        self.current_pose = pose
        self.current_twist = twist

    def pinger_callback(self, msg: Float32MultiArray):
        self.pinger_coordinates = msg.data

    def ready_callback(self, msg: Bool):
        self.ready = msg.data
        if msg.data:
            self.get_logger().info(f'Controller ready')

    def solve_LoS(self, target, current_time):
        x,y,z = target

        yaw_rate = self.k_psi * np.arctan2(y,x)
        d = np.sqrt(x**2+y**2)
        v = self.k_v * d
        v = 2*np.log(v+1)

        thruster_input = [0,0]

        # Convert to differential thrust
        if not self.stopping_sequence:
            if d > 3. :
                thruster_input[0] = v + 0.295 * yaw_rate
                thruster_input[1] = v - 0.295 * yaw_rate
            else:
                self.get_logger().info("LoS target reached, initializing stopping sequence")
                self.stopping_sequence = True
                self.stopping_time = current_time

        # As a safety, if the target is close enough, briefly move back (otherwise te robot will still glide to position) then stop
        else: 
            if current_time - self.stopping_time < 1.0:
                thruster_input = [-1.,-1.]
            else:
                thruster_input = [0.,0.]

        self.get_logger().info(f"Pinger coordinates (robot frame): \n{x}, {y}, {z}")
        self.get_logger().info(f"Computed thrust: \n{thruster_input[0]}, {thruster_input[1]}")

        return thruster_input

    def inRobotFrame(self, robot_coords, target_coords):

        def wrap_angle(angle):
            return (angle + np.pi) % (2 * np.pi) - np.pi

        x_r,y_r,psi_r,_,_,_ = robot_coords
        x_t,y_t,psi_t,_,_,_ = target_coords

        cos = np.cos
        sin = np.sin

        x = (x_t - x_r)*cos(psi_r) + (y_t - y_r)*sin(psi_r)
        y = (y_t - y_r)*cos(psi_r) - (x_t - x_r)*sin(psi_r)
        psi = wrap_angle(psi_t) - wrap_angle(psi_r)

        return x,y,psi

    def timer_callback(self):
        if not self.ready:
            return

        if not self.init:
            if self.controller_type == 'MPC':
                self.controller = ur_mpc.MPCController(robot_mass = 16.01,
                                                iz = 5.64,    # Yaw inertia
                                                a_u = -26.77, # added mass XdotU
                                                a_v = -7.55,  # added mass YdotV
                                                a_r = -21.77, # added mass NdotR
                                                d_u = -29.34, # viscous drag Xu
                                                d_v = -51.54, # viscous drag Yv
                                                d_r = -44.65, # viscous drag Nr
                                                horizon = self.mpc_horizon, 
                                                time = self.mpc_time, 
                                                Q_weight = self.Q_weight,
                                                R_weight = self.R_weight,
                                                input_bounds = self.input_bounds
                                                )

            if self.controller_type == 'PID':
                self.controller = PID.PIDLoS(dt = self.dt,
                                             B = self.B_matrix,
                                             outer_gains = self.outer_gains,
                                             inner_gains = self.inner_gains,
                                             thruster_limits = self.thruster_limits
                                             )

        self.init = True

        if not self.time_set:
            self.initial_time = time.time()
            self.time_set = True
        
        current_time = time.time() - self.initial_time

        ## Update path
        # Check if previous future is still pending
        if self.future is not None:
            if self.future.done():
                try:
                    result = self.future.result()
                    if result is not None:
                        self.controller_path = result.path
                    else:
                        self.get_logger().error("Service returned None.")
                except Exception as e:
                    self.get_logger().error(f"Service call raised exception: {e}")
                finally:
                    self.future = None
                return

        # Send new request
        request = RequestPath.Request()
        request.path_request.data = np.linspace(current_time, current_time + self.path_time, int(self.path_steps), dtype=float)

        self.future = self.client.call_async(request)

        ## Compute thrust
        # Thruster input
        u = [0]*2

        if self.current_pose is not None and self.current_twist is not None:
            current_state = np.array([self.current_pose[0], # x
                                  self.current_pose[1], # y
                                  self.current_pose[5], # yaw
                                  self.current_twist[0], # u
                                  self.current_twist[1], # v
                                  self.current_twist[5]]) # r

        current_state = np.array(current_state).reshape(-1)

        if self.controller_path.poses: # Make sure the path is not empty
            # Display the current desired pose if using gazebo
            if self.isSimulation:
                desired_pose = self.controller_path.poses[0].pose
                cf.create_pose_marker(desired_pose, self.pose_arrow_publisher) 

            if self.controller_type == 'MPC':
                u = self.controller.solve(path=self.controller_path, x_current=current_state)

            if self.controller_type == 'PID':
                target = cf.compute_target(self.controller_path, self.dt)
                u,_ = self.controller.compute(current_state, target[:3])

            if self.controller_type == 'LoS':
                target = cf.compute_target(self.controller_path, self.dt)
                target = self.inRobotFrame(current_state, target)
                u = self.solve_LoS(target, current_time)
        
        elif self.use_pinger and self.pinger_target is not None: # MPC is not supported for this
            if self.controller_type == 'PID':
                # Adapt the controller input to be used in robot frame
                target = [*self.pinger_target, 0]
                current_state[[0,1,2]] = 0
                u,_ = self.controller.compute(current_state, target)

            if self.controller_type == 'LoS':
                target = self.pinger_target
                u = self.solve_LoS(target, current_time)

        # Publish controller target (for data recording)
        if self.use_pinger and self.pinger_target is not None:
            msg = Float32MultiArray()
            msg.data = target
            self.thruster_input_publisher.publish(msg)

        # Publish thruster input
        msg = Float32MultiArray()
        msg.data = u
        self.thruster_input_publisher.publish(msg)
        
        # Update and save monitoring metrics to be graphed later
        if self.controller_path.poses:
            x_m = current_state[0]
            y_m = current_state[1]
            psi_m = current_state[2]

            x_d_m = desired_pose.position.x
            y_d_m = desired_pose.position.y

            q = desired_pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            psi_d_m = math.atan2(siny_cosp, cosy_cosp)

            data_array = [current_time, x_m, y_m, psi_m, x_d_m, y_d_m , psi_d_m, u[0],u[1]]

            self.monitoring.append(data_array)

            publisher_msg = Float32MultiArray()
            publisher_msg.data = data_array
            self.data_publisher.publish(publisher_msg)
            # self.get_logger().info(f'Publishing: {msg.data}')

            if (current_time - self.t_record) > 0.1: # Update the saved file at set interval as doing so every step may corrupt the file if the callback is too frequent
                self.t_record = current_time
                np.save(self.title, self.monitoring)
        

rclpy.init()
node = Controller()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()