# kinematic_core

## Overview
`kinematic_core` translates body-frame velocity commands into individual wheel speeds for a differential-drive robot. It bridges the teleoperation layer and the motor controller.

## Reference Control Flow Diagram
![Control-flow from teleoperation to motor controller](../../docs/figures/control_flow_arcitecture.jpg)

The node follows the control flow above: teleoperation publishes `/r1/cmd_vel`, `kinematic_core` scales the translational and rotational components via gains `k1`–`k4`, and the resulting wheel velocities are forwarded to `/r1/wheel_vel` for the motor controller. Use this diagram as the baseline when you change topics or tune gains.

## Node: `kinematic_node`
Implementation: [`kinematic_core/kinematic_core/movebase_kinematics_diff.py`](kinematic_core/movebase_kinematics_diff.py)

The node subscribes to `/r1/cmd_vel` (`geometry_msgs/msg/Twist`) and publishes `/r1/wheel_vel` (`std_msgs/msg/Float32MultiArray`). Inside `move_cmd_callback`, the script separates the translational and rotational parts of the incoming `Twist`, applies simple gains (`k1`–`k4`), and outputs left/right wheel velocities as a list.

This file is a good starting point for beginners to tweak or extend kinematics: adjust the gains or replace the equations with a full differential-drive model if needed.

## Launch / Usage
Run with `ros2 run kinematic_core kinematic_node`, or rely on the `robot_launch` package to bring the whole chain up.

## Notes for Beginners
- `vel_msg.data = [vel_left, vel_right]` shows how to pack multiple float values into a `Float32MultiArray`.
- The gains mirror the ones annotated in the control-flow diagram above; replace them with values derived from the real robot once those parameters are measured.
