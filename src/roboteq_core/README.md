# roboteq_core

## Overview
`roboteq_core` streams wheel velocity commands to a Roboteq motor controller over a serial connection and monitors the battery voltage. It completes the command pipeline by converting abstract wheel speeds into controller-specific commands, while periodically publishing battery state information.

## Node: `roboteq_node`
Implementation: [`roboteq_core/roboteq_core/roboteq_driver.py`](roboteq_core/roboteq_driver.py)

Key behaviours:
- Opens a serial port (default `/dev/ttyACM0` at 115200 baud) in `setup_serial`.
- Subscribes to `/r1/wheel_vel` (`std_msgs/msg/Float32MultiArray`).
- In `vel_callback`, extracts left/right speeds and forwards them to `move_motors`.
- `move_motors` formats the commands expected by Roboteq controllers (`!G 1 <value>_`).
- Publishes `sensor_msgs/msg/BatteryState` on `/r1/battery_state` at 1 Hz.
- `read_battery_voltage` sends the `?V` query to the controller and parses the response (e.g. `V=135:246:4730`) to extract the battery voltage in volts.
- `battery_monitoring_callback` is fired by a timer, reads the voltage via `read_battery_voltage`, and publishes a `BatteryState` message with thread-safe locking.

Beginners can modify the port, baud rate, or command format by changing the defaults in `setup_serial` and `move_motors`. The battery polling interval can be adjusted via `battery_interval` in `__init__`.

## Usage Tips
- Ensure your user has permission to access the serial device (`/dev/ttyACM0`).
- If running without hardware, comment out the serial write lines or mock the serial port for testing.
- Battery voltage is read from the second field of the `?V` response and converted from deci-volts to volts (e.g. `246` → `24.6 V`).

## Launch
`roboteq_node` is brought up automatically by the `robot_launch` package. You can also run it directly with `ros2 run roboteq_core roboteq_node` once the serial device is connected.
