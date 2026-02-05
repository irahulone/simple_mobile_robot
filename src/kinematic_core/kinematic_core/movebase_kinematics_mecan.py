"""Convert body-frame cmd_vel (vx, vy, wz) into 4 mecanum wheel commands via a matrix."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np


class MecanumKinematicsNode(Node):
    def __init__(self):
        super().__init__("mecanum_kinematics_node")

        # Tunable gains
        self.k_vx = 1.0
        self.k_vy = 1.0
        self.k_wz = 1.0

        # Mixing matrix for [FL, FR, RL, RR]^T = M * [vx, vy, wz]^T
        # Signs assume standard mecanum roller orientation.
        self.M = np.array([
            [ 1.0, -1.0, -1.0],  # FL
            [ 1.0,  1.0,  1.0],  # FR
            [ 1.0,  1.0, -1.0],  # RL
            [ 1.0, -1.0,  1.0],  # RR
        ], dtype=float)

        self.subscription = self.create_subscription(
            Twist, "/r1/cmd_vel", self.cmd_vel_callback, 5
        )
        self.pub_wheels = self.create_publisher(Float32MultiArray, "/r1/wheel_vel", 5)

    def cmd_vel_callback(self, msg: Twist):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)

        v = np.array([self.k_vx * vx, self.k_vy * vy, self.k_wz * wz], dtype=float)
        w = self.M @ v  # [fl, fr, rl, rr]

        out = Float32MultiArray()
        out.data = w.tolist()
        self.pub_wheels.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
