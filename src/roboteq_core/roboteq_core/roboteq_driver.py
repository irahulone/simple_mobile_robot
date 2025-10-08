"""ROS 2 node that forwards wheel velocities to a Roboteq motor controller."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class RoboteqNode(Node):
    """Handle serial communication with a Roboteq controller based on wheel targets."""

    def __init__(self):
        super().__init__('cmd_roboteq')
        self.setup_serial()
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/r1/wheel_vel',
            self.vel_callback,
            5)
        self.subscription  # prevent unused variable warning
    
    def setup_serial(self, port='/dev/ttyACM0', baudrate=115200):
        """Initialize the serial connection to the motor controller."""
        try:
            self.roboteq_obj = serial.Serial(
                port=port,
                baudrate=baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            self.get_logger().info(f'Successfully connected to motor controller on {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to motor controller: {e}')
            raise

    def move_motors(self, vel_left, vel_right):
        """Send velocity commands to each motor channel."""
        payload1 = "!G 1 " + str(int(vel_left)) + "_"
        payload2 = "!G 2 " + str(-int(vel_right)) + "_"
        self.roboteq_obj.write(str.encode(payload1))
        self.roboteq_obj.write(str.encode(payload2))

    def vel_callback(self, msg):
        """Receive wheel velocity targets from ROS 2 and forward them to the driver."""
        vel_left = msg.data[0]
        vel_right = msg.data[1]
        self.move_motors(vel_left, vel_right)

def main(args=None):
    rclpy.init(args=args)
    roboteq_node = RoboteqNode()
    rclpy.spin(roboteq_node)
    roboteq_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
