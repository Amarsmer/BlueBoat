#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R

class PoseToRPY(Node):
    def __init__(self):
        super().__init__('pose_to_rpy')

        self.subscription = self.create_subscription(Pose,'/blueboat/pose_gt',self.pose_callback,10)

        self.publisher = self.create_publisher(String,'pose_rpy',10)

    def pose_callback(self, msg: Pose):
        # Extract position
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z

        # Extract orientation (quaternion)
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Convert quaternion to roll, pitch, yaw
        rot = R.from_quat([qx, qy, qz, qw])
        roll, pitch, yaw = rot.as_euler('xyz', degrees=False)

        # Format output string
        msg_text = f"\nx : {x:.3f}\ny : {y:.3f}\nz : {z:.3f}\nroll : {roll:.3f}\npitch : {pitch:.3f}\nyaw : {yaw:.3f}"

        string_msg = String()
        string_msg.data = msg_text
        self.publisher.publish(string_msg)
        # self.get_logger().info('Published RPY string.')


def main(args=None):
    rclpy.init(args=args)
    node = PoseToRPY()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
