# sim_robot

## Overview
`sim_robot` simulates a mobile robot by receiving wheel velocities and integrating them into a 2-D pose. It replaces physical hardware so the robot can be visualised in RViz2. Both differential-drive (2-wheel) and omni/mecanum (4-wheel) configurations are supported.

## Node: `sim_robot_node`
Implementation: [`sim_robot/sim_robot_node.py`](sim_robot/sim_robot_node.py)

The node subscribes to `wheel_vel` (`std_msgs/msg/Float32MultiArray`) and automatically detects the drive type from the array length:

- **2 values `[left, right]`** — Differential-drive inverse kinematics using k1–k4 gains (same as `kinematic_core`).
- **4 values `[FL, FR, RL, RR]`** — Omni/mecanum inverse kinematics via the pseudoinverse of the mixing matrix, recovering `vx`, `vy`, and `ω`.

The recovered velocities are integrated (midpoint method with body→world rotation) into a 2-D pose (x, y, θ) and published as `sensor_msgs/msg/JointState` on `joint_states`.

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
