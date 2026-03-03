# kinematic_core

## Overview
`kinematic_core` translates body-frame velocity commands into individual wheel speeds for a differential-drive robot. It bridges the teleoperation layer and the motor controller.

## Reference Control Flow Diagram
![Control-flow from teleoperation to motor controller](../../docs/figures/control_flow_arcitecture.jpg)

The node follows the control flow above: teleoperation publishes `cmd_vel`, `kinematic_core` scales the translational and rotational components via gains `k1`–`k4`, and the resulting wheel velocities are forwarded to `wheel_vel` for the motor controller. Use this diagram as the baseline when you change topics or tune gains.

## Node: `kinematic_node`
Implementation: [`kinematic_core/kinematic_core/movebase_kinematics_diff.py`](kinematic_core/movebase_kinematics_diff.py)

The node subscribes to `cmd_vel` (`geometry_msgs/msg/Twist`) and publishes `wheel_vel` (`std_msgs/msg/Float32MultiArray`). Inside `move_cmd_callback`, the script separates the translational and rotational parts of the incoming `Twist`, applies simple gains (`k1`–`k4`), and outputs left/right wheel velocities as a list.

This file is a good starting point for beginners to tweak or extend kinematics: adjust the gains or replace the equations with a full differential-drive model if needed.

### Parameters
The gains k1–k4 are configurable via ROS 2 parameters (declared with `declare_parameter`). Default values are `1.0`. Override them from the shared config file `robot_launch/config/r1.yaml` or pass them directly at launch time.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `k1` | double | 1.0 | Translational gain, left wheel |
| `k2` | double | 1.0 | Rotational gain, left wheel |
| `k3` | double | 1.0 | Translational gain, right wheel |
| `k4` | double | 1.0 | Rotational gain, right wheel |

## Node: `omni_kinematic_node`
Implementation: [`kinematic_core/kinematic_core/movebase_kinematics_omni.py`](kinematic_core/movebase_kinematics_omni.py)

Converts body-frame `cmd_vel` (vx, vy, wz) into 4 mecanum/omni wheel commands via a mixing matrix. Subscribes to `cmd_vel` (`geometry_msgs/msg/Twist`) and publishes `wheel_vel` (`std_msgs/msg/Float32MultiArray`) with `[FL, FR, RL, RR]` wheel velocities.

### Parameters
| Name | Type | Default | Description |
|------|------|---------|-------------|
| `k_vx` | double | 1.0 | Longitudinal velocity gain |
| `k_vy` | double | 1.0 | Lateral velocity gain |
| `k_wz` | double | 1.0 | Rotational velocity gain |

## Launch / Usage
Run with `ros2 run kinematic_core kinematic_node` (diff) or `ros2 run kinematic_core omni_kinematic_node` (omni), or rely on the `robot_launch` package to bring the whole chain up. Pass `kinematic_type:=omni` to the launch file to select the omni model. In simulation, `robot_launch` loads the gains automatically from the shared config.

## Notes for Beginners
- `vel_msg.data = [vel_left, vel_right]` shows how to pack multiple float values into a `Float32MultiArray`.
- The gains mirror the ones annotated in the control-flow diagram above; replace them with values derived from the real robot once those parameters are measured.
