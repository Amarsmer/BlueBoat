#!/usr/bin/env python3

# Common libraries import
import time
from datetime import datetime
import numpy as np
import pandas as pd
from scipy.interpolate import PchipInterpolator
from scipy.spatial.transform import Rotation as R

# ROS2 import
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# msg import
from std_msgs.msg import String, Bool, Float32MultiArray
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State

# srv import
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.srv import CommandLong

# Custom imports
import custom_functions as cf

class BlueBoatController(Node):

    def __init__(self):
        super().__init__('blueboat_controller')

        self.declare_parameter('enable_motors', False)
        self.enable_motors = self.get_parameter('enable_motors').get_parameter_value().bool_value

        ################## ROS2 Communication ##################
        # Publishers
        self.cmd_display = self.create_publisher(OverrideRCIn,'/cmd_display',10)
        self.param_publisher = self.create_publisher(String, '/blueboat/param_str',10)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.str_input_subscriber = self.create_subscription(String, '/blueboat/input_str', self.str_input_callback, 10)
        self.ready_sub = self.create_subscription(Bool,'/blueboat/param_ready',self.param_callback,10)
        self.mode_sub = self.create_subscription(String, '/param_mode',self.mode_callback,10)
        self.state_sub = self.create_subscription(State,'/mavros/state',self.state_callback,10)
        self.imu_sub = self.create_subscription(Imu,'/mavros/imu/data', self.imu_callback, qos)
        self.uw_gps_sub = self.create_subscription(Float32MultiArray,'/uw_gps_data', self.uw_gps_callback,10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')

        ################## Initialize ##################
        self.state = State()

        self.orientation = None
        self.angular_velocity = None
        self.linear_acceleration = None

        self.pinger_coordinates = [0,0,0]
        self.target = [0,0]
        self.thruster_input = [0,0]

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.sent_mode = False
        self.sent_arm = False
        self.init = False

        self.time_set = False

        self.parameter_override = False
        self.mode = ''

        ################## Initialize PWM control (fugly but will do for now) TODO: save the interpolator and load it instead) ##################
        pwm = np.array([1100,1110,1136,1162,1188,1214,1240,1266,1292,1318,1344,1370,1396,1422,1448,1474,1500,1526,1552,1578,1604,1630,1656,1682,1708,1734,1760,1786,1812,1838,1864,1890,1900])
        thr = 9.80665*np.array([-2.81,-2.78,-2.64,-2.42,-2.21,-2.04,-1.83,-1.57,-1.42,-1.2,-0.98,-0.82,-0.6,-0.41,-0.24,-0.09,0,0.21,0.5,0.82,1.17,1.58,1.93,2.37,2.76,3.23,3.57,3.99,4.36,4.84,5.22,5.45,5.63])

        # build inverse interpolator
        self.interpolator = PchipInterpolator(thr, pwm)

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
                             'target_x',
                             'target_y',
                             'right_thr_in',
                             'left_thr_in']

        self.data_size = len(self.data_columns)

        self.uw_gps_log = [0]*self.data_size

        self.df_log = pd.DataFrame(np.zeros(self.data_size).reshape(1, self.data_size),
                                  columns=self.data_columns)

        self.date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')
        self.path = 'data/Robot_data/' + self.date + '-poslog.csv'

    def thrust_to_pwm(self, T): # Thrust in Newton
        return int(self.interpolator(T))

    def set_servo(self, n, pwm):
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
        if self.enable_motors:
            # Sanitize input
            max_input = 50.
            min_input = -20.
            right = np.clip(-input[0], min_input, max_input)
            left = np.clip(input[1], min_input, max_input)

            # Convert thrust to PWM (double sanitation)
            max_PWM = 1900
            min_PWM = 1100
            right_pwm = np.clip(self.thrust_to_pwm(right), min_PWM, max_PWM)
            left_pwm = np.clip(self.thrust_to_pwm(left), min_PWM, max_PWM)

            self.set_servo(1, right_pwm)
            self.set_servo(3, left_pwm)

    def get_time(self):
        s,ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns*1e-9

    def str_input_callback(self, msg: String):
        if msg.data == 'enable':
            self.enable_motors = True

        if msg.data == 'stop':
            self.manualMove([0,0])
            self.setArmedStatus(False) 
            self.enable_motors = False

        self.get_logger().info(f" Enable motors: {self.enable_motors}")

    def param_callback(self, msg: Bool):
        self.parameter_override = msg.data
        self.get_logger().info(f" Param ready: {self.parameter_override}")

    def mode_callback(self, msg: String):
        self.mode = msg.data
        self.get_logger().info(f" Mode received: {self.mode}")

    def state_callback(self, msg):
        self.state = msg

    def imu_callback(self, msg: Imu):
        # orientation (quaternion)
        self.orientation = msg.orientation
        # angular velocity (rad/s)
        self.angular_velocity = msg.angular_velocity
        # linear acceleration (m/s^2)
        self.linear_acceleration = msg.linear_acceleration

    def uw_gps_callback(self, msg):
        # Compile data from gps, imu, and others
        self.uw_gps_log = msg.data

        if self.linear_acceleration == None:
            return

        df_tmp = pd.DataFrame(np.zeros(self.data_size).reshape(1, self.data_size), columns=self.data_columns)

        df_tmp.iloc[0, :19] = msg.data

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

        df_tmp.iloc[0, 29] = self.target[0]
        df_tmp.iloc[0, 30] = self.target[1]

        df_tmp.iloc[0, 31] = self.thruster_input[0]
        df_tmp.iloc[0, 32] = self.thruster_input[1]
        
        self.df_log = pd.concat([self.df_log, df_tmp])

        self.df_log.to_csv(self.path)

        # Stabilize pinger with IMU data
        temp_pinger_coordinates = msg.data[16:19]
        
        # orientation(quat) = [x, y, z, w]
        r = R.from_quat(self.orientation)

        # extract roll, pitch, yaw
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)

        # rebuild rotation with zero yaw
        r_rp = R.from_euler('xyz', [roll, pitch, 0.0])

        # rotate vector
        self.pinger_coordinates = r_rp.apply(temp_pinger_coordinates)


    def setArmedStatus(self,command):
        if command:
            str = 'Arming'
        else:
            str = 'Disarming'

        self.get_logger().info(f"{str} vehicle...")

        if self.arming_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = command
            self.arming_client.call_async(req)

    def timer_callback(self):

        ################## Initialize robot ##################
        if not self.init:
            # Wait until connected
            if not self.state.connected:
                self.get_logger().info('Waiting for FCU connection...')
                return

            # Set mode
            if self.state.mode != "MANUAL" and not self.sent_mode: 
                self.get_logger().info(f"Current mode: {self.state.mode}, switching to MANUAL")

                if self.mode_client.wait_for_service(timeout_sec=1.0):
                    req = SetMode.Request()
                    req.custom_mode = "MANUAL"
                    self.mode_client.call_async(req)
                return

            self.sent_mode = True

            # Change robot parameters to enable direct control
            msg = String()
            msg.data = 'override'
            self.param_publisher.publish(msg)

            self.init = True

        # Wait for direct control to be enabled
        if self.mode != 'override':
            return

        ################## Control loop ##################

        if not self.time_set:
            self.initial_time = time.time()
            self.time_set = True
        
        current_time = time.time()

        # Test input
        if current_time - self.initial_time < 0.5:
            self.manualMove(np.array([10,0]))

        else:
            self.manualMove([0,0])
        
        """
        ## LoS controller
        # Compute surge and yaw rate
        k_v = 1.
        k_psi = 3.
        L = 0.59

        x,y,z = self.pinger_coordinates
        yaw_rate = k_psi * np.arctan2(y,x)
        d = np.sqrt(x**2+y**2)
        v = k_v * d

        # Convert to differential thrust
        T_r = v + 0.5 * L * yaw_rate
        T_l = v - 0.5 * L * yaw_rate

        self.thruster_input = [T_r,T_l]

        self.manualMove([T_r,T_l])
        """
        
rclpy.init()
node = BlueBoatController()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()