# roboteq_core

## Overview
`roboteq_core` streams wheel velocity commands to a Roboteq motor controller over a serial connection. It completes the command pipeline by converting abstract wheel speeds into controller-specific commands.

## Node: `roboteq_node`
Implementation: [`roboteq_core/roboteq_core/roboteq_driver.py`](roboteq_core/roboteq_driver.py)

Key behaviours:
- Opens a serial port (default `/dev/ttyACM0` at 115200 baud) in `setup_serial`.
- Subscribes to `/r1/wheel_vel` (`std_msgs/msg/Float32MultiArray`).
- In `vel_callback`, extracts left/right speeds and forwards them to `move_motors`.
- `move_motors` formats the commands expected by Roboteq controllers (`!G 1 <value>_`).

Beginners can modify the port, baud rate, or command format by changing the defaults in `setup_serial` and `move_motors`.

## Usage Tips
- Ensure your user has permission to access the serial device (`/dev/ttyACM0`).
- If running without hardware, comment out the serial write lines or mock the serial port for testing.

## Launch
`roboteq_node` is brought up automatically by the `robot_launch` package. You can also run it directly with `ros2 run roboteq_core roboteq_node` once the serial device is connected.
