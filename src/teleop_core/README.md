# teleop_core

## Overview
`teleop_core` converts joystick input into velocity commands for the robot. The package exposes a single ROS 2 node and a launch file that can be combined with other packages in this workspace.

## Node: `teleop_node`
Implementation: [`teleop_core/teleop_core/run_joy.py`](teleop_core/run_joy.py)

This node subscribes to `sensor_msgs/msg/Joy` messages, reads the configured axes, and publishes:
- `geometry_msgs/msg/Twist` on `/r1/cmd_vel` for robot motion.
- `std_msgs/msg/Bool` on `/r1/enable` to toggle the drive system.

Default mapping (you can edit the constants near the top of `run_joy.py`):
- Left stick vertical (`axes[1]`) → forward / backward speed (`linear.x`).
- Left stick horizontal (`axes[0]`) → rotation (`angular.z`).
- Left bumper (`buttons[4]`) must be held to enable motion.

Beginners can explore the code by looking at the `joy_callback` function: it reads the joystick message, checks the enable button, and fills a `Twist` message before publishing.

## Launch
Use `ros2 run teleop_core teleop_node` to start the node, or launch everything through `robot_launch`.

## Setup
Ensure the `joy` package is running (`ros2 run joy joy_node`) so that joystick data is available.
