# robot_launch

## Overview
`robot_launch` provides a modular launch system for the robot. A single base launch file (`bringup.launch.py`) exposes per-node toggle flags, and preset launch files (`sim.launch.py`, `actual.launch.py`) offer convenient one-command startup for common configurations.

All nodes are placed under a `/{robot_id}/` namespace so that multiple robots can coexist on the same ROS 2 network.

## Launch Files

### `bringup.launch.py` (base)
Toggle each node individually via launch arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_id` | `r1` | Namespace and config file selection |
| `use_joy` | `false` | Launch `virtual_joy` node |
| `use_teleop` | `true` | Launch `teleop_node` |
| `use_kinematic` | `true` | Launch `kinematic_node` |
| `kinematic_type` | `diff` | Kinematic model: `diff` or `omni` |
| `use_sim` | `false` | Launch `sim_robot_node` |
| `use_actual` | `false` | Launch `roboteq_node` |
| `use_rviz` | `false` | Launch RViz2 |
| `use_description` | `false` | Launch `robot_state_publisher` + static TF |

```bash
ros2 launch robot_launch bringup.launch.py use_sim:=true use_rviz:=true use_description:=true
```

### `sim.launch.py` (simulation preset)
Starts the full simulation stack: virtual joystick, teleop, kinematics, sim_robot, URDF/TF, and RViz2.

```bash
ros2 launch robot_launch sim.launch.py
```

### `actual.launch.py` (hardware preset)
Starts the real-robot stack: teleop, kinematics, and the Roboteq motor driver.

```bash
ros2 launch robot_launch actual.launch.py
```

### Running a different robot
Pass the `robot_id` argument to use a different namespace and config file:

```bash
ros2 launch robot_launch sim.launch.py robot_id:=r2
```

### Omni/mecanum kinematics
Pass `kinematic_type:=omni` to use the omni/mecanum kinematic model instead of the default differential-drive model. The URDF mesh is also automatically switched to `omni_cad.stl`:

```bash
ros2 launch robot_launch sim.launch.py kinematic_type:=omni
```

## Config Files
- `config/r1.yaml` — differential-drive and omni/mecanum gain parameters for robot `r1`.
