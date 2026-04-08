#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool

from mavros_msgs.srv import ParamPull
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue


class BlueBoatParameterControl(Node):
    def __init__(self):
        super().__init__('blueboat_parameter_control')

        ################## ROS2 Communication ##################
        # publishers
        self.ready_pub = self.create_publisher(Bool, '/blueboat/param_ready', 10)
        self.mode_pub = self.create_publisher(String, '/blueboat/param_mode', 10)

        # subscriber
        self.sub = self.create_subscription(String,'/blueboat/param_str',self.callback,10)

        # services
        self.pull_client = self.create_client(ParamPull, '/mavros/param/pull')
        self.get_client = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.set_client = self.create_client(SetParameters, '/mavros/param/set_parameters')

        # state
        self.params_ready = False
        self.current_mode = None
        self.pending_mode = None
        self.target = (0, 0)

        self.wait_services()

    def wait_services(self):
        for cli in [self.pull_client, self.get_client, self.set_client]:
            while not cli.wait_for_service(timeout_sec=1.0):
                pass


    def callback(self, msg: String):
        mode = msg.data.strip()

        if mode == self.current_mode:
            self.publish_state()
            return

        self.apply_mode(mode)


    def apply_mode(self, mode):
        self.pending_mode = mode

        if mode == "override":
            self.target = (0, 0)
        elif mode == "default":
            self.target = (74, 73)
        else:
            self.get_logger().error(f"Unknown mode: {mode}")
            return

        self.pull_params()


    def pull_params(self):
        req = ParamPull.Request()
        future = self.pull_client.call_async(req)
        future.add_done_callback(self._on_pull_done)

    def _on_pull_done(self, future):
        try:
            if not future.result().success:
                raise RuntimeError("Param pull failed")
        except Exception as e:
            self.get_logger().error(str(e))
            self.params_ready = False
            self.publish_state()
            return

        self._set_servo1()

    def _set_servo1(self):
        future = self._set_param_async("SERVO1_FUNCTION", self.target[0])
        future.add_done_callback(self._on_set1_done)

    def _on_set1_done(self, future):
        try:
            if not future.result().results[0].successful:
                raise RuntimeError("Failed to set SERVO1_FUNCTION")
        except Exception as e:
            self.get_logger().error(str(e))
            self.params_ready = False
            self.publish_state()
            return

        future = self._set_param_async("SERVO3_FUNCTION", self.target[1])
        future.add_done_callback(self._on_set2_done)

    def _on_set2_done(self, future):
        try:
            if not future.result().results[0].successful:
                raise RuntimeError("Failed to set SERVO3_FUNCTION")
        except Exception as e:
            self.get_logger().error(str(e))
            self.params_ready = False
            self.publish_state()
            return

        future = self._get_param_async(["SERVO1_FUNCTION", "SERVO3_FUNCTION"])
        future.add_done_callback(self._on_verify_done)

    def _on_verify_done(self, future):
        try:
            values = future.result().values
            val1 = values[0].integer_value
            val3 = values[1].integer_value

            success = (val1 == self.target[0] and val3 == self.target[1])

            self.params_ready = success
            if success:
                self.current_mode = self.pending_mode

        except Exception as e:
            self.get_logger().error(str(e))
            self.params_ready = False

        self.publish_state()


    def _set_param_async(self, name, value):
        param = Parameter()
        param.name = name

        val = ParameterValue()
        val.type = 2  # integer
        val.integer_value = int(value)

        param.value = val

        req = SetParameters.Request()
        req.parameters = [param]

        return self.set_client.call_async(req)

    def _get_param_async(self, names):
        req = GetParameters.Request()
        req.names = names
        return self.get_client.call_async(req)

    def publish_state(self):
        ready_msg = Bool()
        ready_msg.data = self.params_ready
        self.ready_pub.publish(ready_msg)

        if self.current_mode is not None:
            mode_msg = String()
            mode_msg.data = self.current_mode
            self.mode_pub.publish(mode_msg)

rclpy.init()
node = BlueBoatParameterControl()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()