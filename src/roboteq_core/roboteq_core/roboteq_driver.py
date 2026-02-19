"""ROS 2 node that forwards wheel velocities to a Roboteq motor controller."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import BatteryState
import serial
import threading

class RoboteqNode(Node):
    """Handle serial communication with a Roboteq controller based on wheel targets."""

    def __init__(self):
        super().__init__('cmd_roboteq')
        self.setup_serial()
                
        # Battery monitoring parameters
        self.battery_interval = 1.0
        
        # Battery monitoring state
        self.last_battery_voltage = 0.0
        self.battery_lock = threading.Lock()
    
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/r1/wheel_vel',
            self.vel_callback,
            5)
        self.subscription  # prevent unused variable warning
        self.battery_publisher = self.create_publisher(
            BatteryState,
            f'/r1/battery_state',
        5)
        # Battery monitoring timer
        self.battery_timer = self.create_timer(
            self.battery_interval,
            self.battery_monitoring_callback)
    
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


    def read_battery_voltage(self):
        """Read battery voltage from Roboteq controller using ?V command"""
        try:
            # Send voltage query command
            query_cmd = "?V_"
            self.roboteq_obj.write(str.encode(query_cmd))
            
            # Read response with timeout
            response = self.roboteq_obj.readline().decode('utf-8').strip()
            
            # Parse response (expected format: "V=135:246:4730")
            if response:
                # Remove any trailing underscore and parse
                response = response.replace('_', '')
                if '=' in response:
                    voltage_data = response.split('=')[1]
                    
                    # Split by colon to get multiple voltage values
                    voltage_parts = voltage_data.split(':')
                    if len(voltage_parts) >= 2:
                        # Use second value as battery voltage (index 1)
                        # Convert from centi-volts to volts (246 -> 24.6V)
                        battery_voltage = float(voltage_parts[1]) / 10.0
                        
                        self.get_logger().debug(f'Raw voltage response: {response}')
                        self.get_logger().debug(f'Internal: {float(voltage_parts[0])/10.0:.1f}V, Battery: {battery_voltage:.1f}V, 5V: {float(voltage_parts[2])/10.0:.1f}V' if len(voltage_parts) >= 3 else f'Voltages: {voltage_parts}')
                        
                        return battery_voltage
                    else:
                        self.get_logger().debug(f'Insufficient voltage values in response: {response}')
                        return None
                else:
                    self.get_logger().debug(f'Invalid response format: {response}')
                    return None
            return None
            
        except (serial.SerialException, ValueError, IndexError) as e:
            self.get_logger().debug(f'Error reading battery voltage: {e}')
            return None

    def battery_monitoring_callback(self):
        """Timer callback for battery voltage monitoring"""
        try:
            with self.battery_lock:
                voltage = self.read_battery_voltage()
                
                if voltage is not None:
                    # Round voltage to 1 decimal place for consistent display
                    voltage_rounded = round(voltage, 1)
                    self.last_battery_voltage = voltage_rounded
                    
                    # Create and publish BatteryState message
                    battery_msg = BatteryState()
                    battery_msg.header.stamp = self.get_clock().now().to_msg()
                    battery_msg.header.frame_id = "r1_battery"
                    
                    # Voltage information
                    battery_msg.voltage = voltage_rounded
                    battery_msg.present = True
                    
                    # Power supply health (always good for simple monitoring)
                    battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
                    battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
                    battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
                    
                    # Unknown values (set to NaN or 0 as appropriate)
                    battery_msg.current = float('nan')
                    battery_msg.charge = float('nan')
                    battery_msg.capacity = float('nan')
                    battery_msg.percentage = float('nan')
                    battery_msg.design_capacity = float('nan')
                    
                    self.battery_publisher.publish(battery_msg)
                    self.get_logger().debug(f'Battery voltage: {voltage_rounded:.1f}V')
                else:
                    self.get_logger().debug('Failed to read battery voltage')
                    
        except Exception as e:
            self.get_logger().error(f'Battery monitoring error: {e}')


def main(args=None):
    rclpy.init(args=args)
    roboteq_node = RoboteqNode()
    rclpy.spin(roboteq_node)
    roboteq_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
