"""Convert body-frame velocity commands into differential wheel speeds."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class KinematicDiffNode(Node):
    """Compute left/right wheel velocities from linear and angular motion commands."""

    def __init__(self):
        super().__init__('movebase_kinematics_node')
        self.k1 = 1.0
        self.k2 = 1.0
        self.k3 = 1.0
        self.k4 = 1.0

        self.subscription = self.create_subscription(
            Twist,
            '/r1/cmd_vel',
            self.move_cmd_callback,
            5)
        self.subscription  # prevent unused variable warning

        self.pub_velocity = self.create_publisher(Float32MultiArray, '/r1/wheel_vel', 5)

    def move_cmd_callback(self, msg):
        """Translate a `Twist` command into wheel speeds and publish the result."""
        vel_translational = msg.linear.x
        vel_rotational = msg.angular.z

        vel_left = self.k1 * vel_translational - self.k2 * vel_rotational
        vel_right = self.k3 * vel_translational + self.k4 * vel_rotational

        vel_msg = Float32MultiArray()
        vel_msg.data = [vel_left, vel_right]
        self.pub_velocity.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)

    kinematic_node = KinematicDiffNode()
    rclpy.spin(kinematic_node)
    kinematic_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
