#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from mavros_msgs.msg import OverrideRCIn


class BlueBoatController(Node):

    def __init__(self):
        super().__init__('blueboat_controller')

        self.declare_parameter('enable_motors', False)
        self.enable_motors = self.get_parameter('enable_motors').get_parameter_value().bool_value

        ################## ROS2 Communication ##################
        # Publishers
        self.cmd_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        # self.cmd_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.cmd_display = self.create_publisher(OverrideRCIn,'/cmd_display',10)

        # Subscribers
        self.state_sub = self.create_subscription(State,'/mavros/state',self.state_callback,10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        ################## Initialize ##################
        self.state = State()

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.sent_mode = False
        self.sent_arm = False

        self.time_set = False

        self.get_logger().info("Node initialized")

    def get_time(self):
        s,ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns*1e-9

    def state_callback(self, msg):
        # self.get_logger().info(f"STATE RECEIVED: \nconnected={msg.connected} \nmode={msg.mode}")
        self.state = msg

    def timer_callback(self):
        # Wait until connected
        if not self.state.connected:
            self.get_logger().info('Waiting for FCU connection...')
            return

        # Set mode
        if self.state.mode != "MANUAL":  
            self.get_logger().info(f"Current mode: {self.state.mode}, switching to MANUAL")

            if self.mode_client.wait_for_service(timeout_sec=1.0):
                req = SetMode.Request()
                req.custom_mode = "MANUAL"
                self.mode_client.call_async(req)
            return

        # Arm
        if not self.state.armed:
            self.get_logger().info("Arming vehicle...")

            if self.arming_client.wait_for_service(timeout_sec=1.0):
                req = CommandBool.Request()
                req.value = True
                self.arming_client.call_async(req)
            return

        if not self.time_set:
            self.initial_time = time.time()
            self.time_set = True
        
        current_time = time.time()
        # Publish velocity
        cmd = OverrideRCIn()

        # initialize all channels to "ignore"
        cmd.channels = [0] * 18

        # if current_time - self.initial_time < 0.5:
        #     cmd.linear.x = 1.0  # forward
        #     cmd.angular.z = 1.0  # turn
        # else:
        #     cmd.linear.x = 0.0   # forward
        #     cmd.angular.z = 0.0 # turn

        if current_time - self.initial_time < 0.5:
            right_pwm = 1600
            left_pwm  = 1400

            cmd.channels[0] = right_pwm   # channel 1
            cmd.channels[2] = left_pwm    # channel 3
        else:
            right_pwm = 1500
            left_pwm  = 1500

            cmd.channels[0] = right_pwm   # channel 1
            cmd.channels[2] = left_pwm    # channel 3

        # self.get_logger().info(f"Current command: {cmd}")
        # self.get_logger().info(f"Recorded time: {current_time-self.initial_time}")

        self.cmd_display.publish(cmd)
        if self.enable_motors:
            self.cmd_pub.publish(cmd)

rclpy.init()
node = BlueBoatController()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()

