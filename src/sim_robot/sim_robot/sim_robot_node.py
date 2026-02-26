"""Simulate a differential-drive robot by integrating wheel velocities into 2-D pose."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class SimRobotNode(Node):
    """Receive wheel velocities, apply inverse kinematics, and publish joint states."""

    def __init__(self):
        super().__init__('sim_robot_node')

        # Kinematic gains (same as kinematic_core)
        self.declare_parameter('k1', 1.0)
        self.declare_parameter('k2', 1.0)
        self.declare_parameter('k3', 1.0)
        self.declare_parameter('k4', 1.0)
        self.k1 = self.get_parameter('k1').value
        self.k2 = self.get_parameter('k2').value
        self.k3 = self.get_parameter('k3').value
        self.k4 = self.get_parameter('k4').value

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'wheel_vel',
            self.wheel_vel_callback,
            5,
        )

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 5)

    def wheel_vel_callback(self, msg):
        """Inverse kinematics → midpoint integration → publish joint states."""
        if len(msg.data) < 2:
            return

        vel_left = msg.data[0]
        vel_right = msg.data[1]

        # Inverse kinematics: recover v and omega from wheel velocities
        det = self.k1 * self.k4 + self.k2 * self.k3
        v = (self.k4 * vel_left + self.k2 * vel_right) / det
        omega = (-self.k3 * vel_left + self.k1 * vel_right) / det

        # Time step
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        # Midpoint integration
        theta_mid = self.theta + 0.5 * omega * dt
        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt
        self.theta += omega * dt

        # Publish joint states for robot_state_publisher
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ['w_to_x', 'x_to_y', 'y_to_t']
        js.position = [self.x, self.y, self.theta]
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = SimRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
