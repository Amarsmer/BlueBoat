#!/usr/bin/env python3

# Common libraries import
import time
from datetime import datetime
import numpy as np
import pandas as pd
import math
from scipy.spatial.transform import Rotation as R
import transformations as tf_transformations

# ROS2 import
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# msg import
from std_msgs.msg import String, Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu, NavSatFix
from mavros_msgs.msg import State

# srv import
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.srv import CommandLong

# Custom imports
import custom_functions as cf

class BlueBoatController(Node):

    def __init__(self):
        super().__init__('blueboat_controller')

        ################## Get Parameters ##################
        self.declare_parameter('enable_motors', False)
        self.enable_motors = self.get_parameter('enable_motors').get_parameter_value().bool_value

        self.declare_parameter('note', '')
        self.note = self.get_parameter('note').get_parameter_value().string_value

        self.declare_parameter('controller_type', '') 
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value

        ################## ROS2 Communication ##################
        ## Publishers
        self.param_publisher = self.create_publisher(String, '/blueboat/param_str',10)
        self.odom_publisher = self.create_publisher(Odometry, '/blueboat/odom',10)
        self.pinger_publisher = self.create_publisher(Float32MultiArray, '/blueboat/pinger_coordinates', 10)
        self.set_controller_publisher = self.create_publisher(Bool, '/blueboat/controller_ready',10)
        self.plot_publisher = self.create_publisher(Float32MultiArray, "blueboat/monitoring_data", 10)

        ## Subscribers
        # Node interaction
        self.str_input_subscriber = self.create_subscription(String, '/blueboat/input_str', self.str_input_callback, 10)
        self.ready_sub = self.create_subscription(Bool,'/blueboat/param_ready',self.param_callback,10)
        self.mode_sub = self.create_subscription(String, '/blueboat/param_mode',self.mode_callback,10)

        # Robot sensor
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.robot_state_sub = self.create_subscription(State,'/mavros/state',self.state_callback,10)
        self.imu_sub = self.create_subscription(Imu,'/mavros/imu/data', self.imu_callback, qos)
        self.local_odom_sub = self.create_subscription(Odometry, '/mavros/local_position/odom', self.odom_callback, qos)
        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos)

        # Data logging
        self.uw_gps_sub = self.create_subscription(Float32MultiArray,'/uw_gps_data', self.uw_gps_callback,10)
        self.target_sub = self.create_subscription(Float32MultiArray,'/controller_target', self.target_callback,10)
        self.thruster_input_sub = self.create_subscription(Float32MultiArray, "/thruster_input", self.thr_input_callback,10)

        ## Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')

        ################## Initialize ##################
        self.robot_state = State()

        # Main loop initialization variables
        self.init = False
        self.mode = ''

        self.timer = self.create_timer(0.05, self.timer_callback)

        # Manual input control init
        self.stopping_sequence = False
        self.stopping_time = 0.
        self.manual_move_timer = 0.

        self.time_set = False

        ### Sensors and dead reckoning parameters
        # IMU
        self.orientation = None
        self.angular_velocity = None
        self.linear_acceleration = None

        # GPS 
        self.gps_data = [0,0] # latitude, longitude
        self.pinger_gps = [0,0]

        self.prev_time = None
        self.vel = np.zeros(3)
        self.pos = np.zeros(3)
        self.yaw0 = None # Used to start the starting yaw at 0 regardless of actual orientation

        ## Control

        self.relative_coordinates = [0,0,0]
        self.target = [0,0,0]
        self.pinger_coordinates = np.zeros(3)
        self.corrected_pinger = [0,0]
        self.thruster_input = [0,0]

        ################## Initialize PWM control ##################
        self.interpolator = cf.generate_interpolator()

        ################## Initialize data collection ##################

        self.data_columns = ['Year', 
                             'Month', 
                             'Day', 
                             'Hour', 
                             'Minute', 
                             'Second', 
                             'MicroSecond', 
                             'aco_x', 
                             'aco_y', 
                             'aco_z', 
                             'ant_x', 
                             'ant_y', 
                             'ant_z', 
                             'lat', 
                             'lon', 
                             'dep', 
                             'filaco_x', 
                             'filaco_y', 
                             'filaco_z',
                             'quat_x', 
                             'quat_y', 
                             'quat_z', 
                             'quat_w',
                             'ang_vel_x',
                             'ang_vel_y',
                             'ang_vel_z',
                             'lin_acc_x',
                             'lin_acc_y',
                             'lin_acc_z',
                             'relative_x',
                             'relative_y',
                             'relative_psi',
                             'target_x',
                             'target_y',
                             'target_psi',
                             'corrected_pinger_x',
                             'corrected_pinger_y',
                             'gps_latitude',
                             'gps_longitude',
                             'pinger_latitude',
                             'pinger_longitude',
                             'left_thr_in',
                             'right_thr_in']

        self.data_size = len(self.data_columns)

        self.uw_gps_log = [0]*self.data_size

        self.df_log = pd.DataFrame(np.zeros(self.data_size).reshape(1, self.data_size),
                                  columns=self.data_columns)

        self.date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')
        self.path = f'data/Robot_data/{self.date}-{self.note}-poslog.csv'

    ################## Thruster interaction ##################

    def set_servo(self, n, pwm):
        """
        Send a PWM signal to a specific servo (n)
        """
        req = CommandLong.Request()
        req.command = 183
        req.param1 = float(n)
        req.param2 = float(pwm)

        # explicitly set all remaining params as float
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        self.cmd_client.call_async(req)

    def manualMove(self, input):
        """
        Convert a newton input to pwm and send it to motor
        """

        # Safety
        if not self.enable_motors:
            return

        def thrust_to_pwm(T): # Thrust in Newton
            return int(self.interpolator(T))
        
        # Compensate right thruster observed weaker output
        if input[1] >= 0:
            compensation_gain = 1.2
        else:
            compensation_gain = 0.75

        # Sanitize input
        max_input = 20.
        min_input = -20.
        left = np.clip(input[1], min_input, max_input)
        right = np.clip(input[0]*compensation_gain, min_input, max_input)

        # Convert thrust to PWM (double sanitation)
        max_PWM = 1900
        min_PWM = 1100
        right_pwm = np.clip(thrust_to_pwm(right), min_PWM, max_PWM)
        left_pwm = 3000 - np.clip(thrust_to_pwm(left), min_PWM, max_PWM) # Reverses direction of thruster rotation to account for asymmetrical propeller

        # Apply PWM to thrusters
        self.set_servo(1, right_pwm)
        self.set_servo(3, left_pwm)


    ################## User interaction ##################
    def setArmedStatus(self,command):
        """
        Either arm or disarm the robot's thrusters. Note that the 'override' parameter completely disregards armed status
        """
        self.get_logger().info(f"{'Arming' if command else 'Disarming'} vehicle...")

        if self.arming_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = command
            self.arming_client.call_async(req)

    def SetMode(self, mode):
        """
        Set the robot's mode to the requested input.
        """
        self.get_logger().info(f"Current mode: {self.robot_state.mode}, switching to {mode}]")

        if self.mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = mode
            self.mode_client.call_async(req)
    
    def set_motors(self, inBool):
        """
        Set the bool value of enable_motors. 
        This is meant as a safety as no input will be set to the thrusters intil this is set to True
        """
        self.enable_motors = inBool
        self.get_logger().info(f" Enable motors: {self.enable_motors}")

    def full_stop(self):
        """
        Cancels any thruster input and set control parameters to False
        """
        self.manualMove([0,0])
        self.setArmedStatus(False) 
        self.set_motors(False)

    def publish(self, msg_type, in_msg, publisher):
        """
        Makes publishing within code neater
        """
        msg = msg_type
        msg.data = in_msg
        publisher.publish(msg)

    def move_callback(self, in_str):
        """
        Called when input_str is 'move', the first two floats are left and right thruster inputs, 
        the last one is the length (in seconds) of the applied thrust
        """

        # Make sure the command is valid
        if len(in_str) != 4:
            self.get_logger().info(f" Incorrect move command.")
            return

        # Start measuring time and apply thrust
        self.initial_time = time.time()
        left, right, self.manual_move_timer = map(float, in_str[1:])
        self.thruster_input = [right,left]

    def str_input_callback(self, msg: String):
        """
        Read str_msg content and take required action
        """
        input_string = msg.data.split()
        command = input_string[0]
        param_publish = lambda: self.publish(String(), command, self.param_publisher)
        
        dispatch = {'enable': lambda: self.set_motors(True),
                    'stop': self.full_stop,
                    'override': param_publish,
                    'default': param_publish,
                    'move': lambda: self.move_callback(input_string),
        }

        action = dispatch.get(command, lambda: self.move_callback(input_string))
        action()   

    ################## ROS2 node interaction ##################

    def param_callback(self, msg: String):
        """
        Returns true if the parameter changes are successful (used with the 'default' and 'override' command)
        """
        self.get_logger().info(f" Parameters ready: {msg.data}")

    def mode_callback(self, msg: String):
        """
        Displays the mode sent to the robot to confirm the changes
        """
        self.mode = msg.data
        self.get_logger().info(f" Mode received: {self.mode}")

    def state_callback(self, msg):
        """
        Read the state of the robot
        """
        self.robot_state = msg

    def imu_callback(self, msg: Imu):
        self.orientation = msg.orientation                  # (quaternion)
        self.angular_velocity = msg.angular_velocity        # (rad/s)
        self.linear_acceleration = msg.linear_acceleration  # (m/s^2)

    def odom_callback(self, msg: Odometry):
        def quaternion_to_yaw(q: Quaternion):
            # yaw (Z axis rotation)
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)

        def yaw_to_quaternion(yaw: float):
            q = Quaternion()
            q.w = math.cos(yaw * 0.5)
            q.x = 0.0
            q.y = 0.0
            q.z = math.sin(yaw * 0.5)
            return q

        def normalize_angle(angle):
            return math.atan2(math.sin(angle), math.cos(angle))

        def transform_body_to_world(x_r, y_r, yaw, x_b, y_b):
            c = math.cos(yaw)
            s = math.sin(yaw)

            x_w = x_r + (c * x_b - s * y_b)
            y_w = y_r + (s * x_b + c * y_b)

            return x_w, y_w

        def enu_to_gps(lat0_deg, lon0_deg, east, north):
            EARTH_RADIUS = 6378137.0  # meters

            lat0 = math.radians(lat0_deg)
            lon0 = math.radians(lon0_deg)

            dlat = north / EARTH_RADIUS
            dlon = east / (EARTH_RADIUS * math.cos(lat0))

            lat = lat0 + dlat
            lon = lon0 + dlon

            return math.degrees(lat), math.degrees(lon)

        def local_to_enu(x, y, yaw0):
            # rotate local frame into ENU
            theta = yaw0 - math.pi / 2.0

            c = math.cos(theta)
            s = math.sin(theta)

            east  = c * x - s * y
            north = s * x + c * y

            return east, north

        # Set previous time measurement and compute dt
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_time is None:
            self.prev_time = t
            return
        dt = t - self.prev_time
        self.prev_time = t

        # Initialize reference on first callback
        if not hasattr(self, "origin_set") or not self.origin_set:
            self.x0 = msg.pose.pose.position.x
            self.y0 = msg.pose.pose.position.y
            self.z0 = msg.pose.pose.position.z
            self.yaw0 = quaternion_to_yaw(msg.pose.pose.orientation)
            self.lat0 = self.gps_data[0]
            self.lon0 = self.gps_data[1]
            self.origin_set = True

        # Position offset
        x_rel = msg.pose.pose.position.x - self.x0
        y_rel = msg.pose.pose.position.y - self.y0
        z_rel = msg.pose.pose.position.z - self.z0

        # Yaw offset
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)        
        yaw_rel = normalize_angle(yaw - self.yaw0)

        self.relative_coordinates = [x_rel,y_rel,yaw_rel]

        # Build modified odometry
        odom_out = Odometry()
        odom_out.header = msg.header
        odom_out.child_frame_id = msg.child_frame_id

        odom_out.pose.pose.position.x = x_rel
        odom_out.pose.pose.position.y = y_rel
        odom_out.pose.pose.position.z = z_rel
        odom_out.pose.pose.orientation = yaw_to_quaternion(yaw_rel)

        # Preserve velocity and covariance
        odom_out.twist = msg.twist
        odom_out.pose.covariance = msg.pose.covariance
        odom_out.twist.covariance = msg.twist.covariance

        self.odom_publisher.publish(odom_out)

        x_t = msg.twist.twist.linear.x
        y_t = msg.twist.twist.linear.y
        z_t = msg.twist.twist.linear.z
        self.vel = np.array([x_t,y_t,z_t])

        av = self.angular_velocity

        # Apply sensor fusion to get a smoother approximation at higher frequency of pinger_coordinates
        if not all(self.pinger_coordinates == np.zeros(3)): # Make sure the pinger has been detected
            omega = np.array([0.0, 0.0, av.z])
            p = self.pinger_coordinates

            self.pinger_coordinates -= (self.vel + np.cross(omega, p)) * dt
        
        self.publish(Float32MultiArray(), self.pinger_coordinates, self.pinger_publisher)

        if not hasattr(self, "origin_set") or not self.origin_set:
            return  

        # rotate pinger coordinates into original frame
        x_body = self.pinger_coordinates[0]
        y_body = self.pinger_coordinates[1]

        x_world, y_world = transform_body_to_world(x_rel, y_rel, yaw_rel, x_body, y_body)

        self.corrected_pinger = [x_world, y_world]

        # convert local pinger into gps coordinates
        east, north = local_to_enu(x_world,y_world,self.yaw0)

        lat, lon = enu_to_gps(self.lat0, self.lon0, east, north)

        self.pinger_gps = [lat, lon]

    def gps_callback(self, msg : NavSatFix):
        self.gps_data = [msg.latitude, msg.longitude]

    def uw_gps_callback(self, msg):
        """
        Read msg from the underwater_gps node, compile it with robot data and save the log
        """

        # Make sure the robot's data is available
        if self.orientation is None:
            return

        ## Compile data from gps, imu, and others
        self.uw_gps_log = msg.data

        if self.linear_acceleration == None:
            return

        df_tmp = pd.DataFrame(np.zeros(self.data_size).reshape(1, self.data_size), columns=self.data_columns)

        df_tmp.iloc[0, :19] = msg.data

        t_x,t_y,t_z = df_tmp.iloc[0, 16:19]
        self.pinger_coordinates = np.array([t_x,t_y,t_z])

        df_tmp.iloc[0, 19] = self.orientation.x
        df_tmp.iloc[0, 20] = self.orientation.y
        df_tmp.iloc[0, 21] = self.orientation.z
        df_tmp.iloc[0, 22] = self.orientation.w

        df_tmp.iloc[0, 23] = self.angular_velocity.x
        df_tmp.iloc[0, 24] = self.angular_velocity.x
        df_tmp.iloc[0, 25] = self.angular_velocity.x

        df_tmp.iloc[0, 26] = self.linear_acceleration.x
        df_tmp.iloc[0, 27] = self.linear_acceleration.y
        df_tmp.iloc[0, 28] = self.linear_acceleration.z

        df_tmp.iloc[0, 29] = self.relative_coordinates[0]
        df_tmp.iloc[0, 30] = self.relative_coordinates[1]
        df_tmp.iloc[0, 31] = self.relative_coordinates[2]

        df_tmp.iloc[0, 32] = self.target[0]
        df_tmp.iloc[0, 33] = self.target[1]
        df_tmp.iloc[0, 34] = self.target[2]

        df_tmp.iloc[0, 35] = self.corrected_pinger[0]
        df_tmp.iloc[0, 36] = self.corrected_pinger[1]

        df_tmp.iloc[0, 37] = self.gps_data[0]
        df_tmp.iloc[0, 38] = self.gps_data[1]

        df_tmp.iloc[0, 39] = self.pinger_gps[0]
        df_tmp.iloc[0, 40] = self.pinger_gps[1]

        df_tmp.iloc[0, 41] = self.thruster_input[0]
        df_tmp.iloc[0, 42] = self.thruster_input[1]
        
        self.df_log = pd.concat([self.df_log, df_tmp])

        self.df_log.to_csv(self.path)

    def target_callback(self, msg: Float32MultiArray):
        """
        Update the target, used when interacting with the controller node
        """
        self.target = msg.data

    def thr_input_callback(self, msg: Float32MultiArray):
        """
        Update the thruster inputs, used when interacting with the controller node
        """
        self.thruster_input = msg.data

    def timer_callback(self):
        """
        Main loop
        """

        ################## Initialize robot ##################
        if not self.init:
            # Wait until connected
            if not self.robot_state.connected:
                self.get_logger().info('Waiting for FCU connection...')
                return

            # Set mode
            if self.robot_state.mode != "MANUAL": 
                self.SetMode('MANUAL')
                return

            self.publish(String(), 'override', self.param_publisher)

            self.init = True

        # Wait for direct control to be enabled
        if self.mode != 'override':
            return

        ################## Control loop ##################
        
        # Start recording time
        if not self.time_set:
            self.initial_time = time.time()

            # Send ready msg to controller node
            self.publish(Bool(), True, self.set_controller_publisher)

            self.time_set = True
        
        current_time = time.time()
        
        self.get_logger().info(f'Corrected pinger: {self.corrected_pinger}')
        ## Send input to thrusters

        # If no controller is set, allow for manual input
        if self.controller_type == '' and current_time - self.initial_time >= self.manual_move_timer:
            self.manualMove([0, 0])
        else:
            self.manualMove(self.thruster_input)        
        
rclpy.init()
node = BlueBoatController()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()