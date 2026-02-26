# sim_robot

## Overview
`sim_robot` simulates a differential-drive robot by receiving wheel velocities and integrating them into a 2-D pose. It replaces physical hardware so the robot can be visualised in RViz2.

## Node: `sim_robot_node`
Implementation: [`sim_robot/sim_robot_node.py`](sim_robot/sim_robot_node.py)

The node subscribes to `wheel_vel` (`std_msgs/msg/Float32MultiArray`, 2 elements: left, right) and applies inverse kinematics using the same k1–k4 gains as `kinematic_core` to recover translational velocity `v` and rotational velocity `ω`. It then integrates the pose (x, y, θ) with midpoint integration and publishes the result as `sensor_msgs/msg/JointState` on `joint_states`.

### Parameters
| Name | Type | Default | Description |
|------|------|---------|-------------|
| `k1` | double | 1.0 | Translational gain, left wheel |
| `k2` | double | 1.0 | Rotational gain, left wheel |
| `k3` | double | 1.0 | Translational gain, right wheel |
| `k4` | double | 1.0 | Rotational gain, right wheel |

### Topics
| Topic | Type | Direction |
|-------|------|-----------|
| `wheel_vel` | `std_msgs/msg/Float32MultiArray` | Subscribe |
| `joint_states` | `sensor_msgs/msg/JointState` | Publish |

## Launch / Usage
Run standalone with `ros2 run sim_robot sim_robot_node`, or as part of the simulation stack via `ros2 launch robot_launch sim.launch.py`.
