#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State


class BlueBoatController(Node):

    def __init__(self):
        super().__init__('blueboat_controller')

        # Publisher
        self.cmd_pub = self.create_publisher(Twist,'/mavros/setpoint_velocity/cmd_vel',10)
        self.cmd_display = self.create_publisher(Twist,'/cmd_display',10)

        # Subscribers
        self.state_sub = self.create_subscription(State,'/mavros/state',self.state_callback,10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.state = State()

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.sent_mode = False
        self.sent_arm = False

        self.initial_time = self.get_time()

    def get_time(self):
        s,ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns*1e-9

    def state_callback(self, msg):
        # self.get_logger().info(f"STATE RECEIVED: connected={msg.connected}")
        self.state = msg

    def timer_callback(self):

        # Wait until connected
        if not self.state.connected:
            self.get_logger().info('Waiting for FCU connection...')
            return

        self.get_logger().info(f"#\n#\n#\nCurrent mode: {self.state.mode}\n#\n#\n#")

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

        # Publish velocity (continuous!)
        cmd = Twist()

        if self.get_time() - self.initial_time < 0.5:
            cmd.linear.x = 0.5   # forward
            cmd.angular.z = 0.2  # turn
        else:
            cmd.linear.x = 0.   # forward
            cmd.angular.z = 0.  # turn

        self.cmd_display.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BlueBoatController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()