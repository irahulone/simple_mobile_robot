"""ROS 2 node for translating joystick input into velocity commands."""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class JoyNode(Node):
    """Listen to joystick events and publish velocity plus enable commands."""

    def __init__(self):
        super().__init__('teleop_node')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.translational_axis_id = 1
        self.rotational_axis_id = 0
        self.en_button_id = 4
        
        self.publisher_cmd_vel = self.create_publisher(Twist, '/r1/cmd_vel', 5)
        self.publisher_en = self.create_publisher(Bool, '/r1/enable', 1)
    
    def joy_callback(self, msg):
        """Process a `Joy` message and update the robot command outputs."""
        vel_translational = msg.axes[self.translational_axis_id]
        vel_rotational = msg.axes[self.rotational_axis_id]
        vel_msg = Twist()

        en_state = Bool()
        en_state.data = False

        if msg.buttons[self.en_button_id] == 1:
            en_state.data = True
            vel_msg.linear.x = vel_translational
            vel_msg.angular.z = vel_rotational
        else:
            en_state.data = False
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0

        self.publisher_cmd_vel.publish(vel_msg)
        self.publisher_en.publish(en_state)


def main(args=None):
    rclpy.init(args=args)
    joy_handle = JoyNode()
    rclpy.spin(joy_handle)
    joy_handle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
