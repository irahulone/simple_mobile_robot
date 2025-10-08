# robot_launch

## Overview
`robot_launch` collects the nodes provided by the other packages into convenient launch files. It is the easiest entry point for beginners to bring the full stack online.

## Launch File: `robot_launch.launch.py`
Implementation: [`robot_launch/launch/robot_launch.launch.py`](launch/robot_launch.launch.py)

This launch description instantiates three ROS 2 nodes:
1. `teleop_core/teleop_node` – reads the joystick and publishes `/r1/cmd_vel` and `/r1/enable`.
2. `kinematic_core/kinematic_node` – converts body velocities to wheel speeds.
3. `roboteq_core/roboteq_node` – streams the wheel speeds to the Roboteq controller via serial.

Beginners can open the launch file to see how ROS 2 launch actions are composed. The `Node(...)` entries show how to specify `package`, `executable`, and optional console output settings.

## Usage
After building the workspace:
```bash
source install/setup.bash
ros2 launch robot_launch robot_launch.launch.py
```
This command starts all components in the correct order.
