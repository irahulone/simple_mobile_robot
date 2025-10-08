# Simple Mobile Robot

This workspace targets ROS 2 Jazzy and demonstrates how joystick input flows all the way to a motor controller on a differential-drive robot. Each package has its own `README` with more detail for beginners:

- `teleop_core` – joystick to velocity commands ([see README](src/teleop_core/README.md)).
- `kinematic_core` – body velocities to wheel speeds ([see README](src/kinematic_core/README.md)).
- `roboteq_core` – wheel speeds to Roboteq serial commands ([see README](src/roboteq_core/README.md)).
- `robot_launch` – launch files that start the whole stack ([see README](src/robot_launch/README.md)).

If you are new to ROS 2, start by skimming the package READMEs in order; they explain what each node does and how the messages flow between them.

![Control Flow Architecture](docs/figures/control_flow_arcitecture.jpg)

## Quick Start

1. Install ROS 2 Jazzy (Desktop) and source your setup file:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```
2. In a new terminal, clone this repository and build the packages:
   ```bash
   colcon build --packages-select teleop_core kinematic_core roboteq_core robot_launch
   ```
   Optionally add `--symlink-install` during development to mirror files into the install space without rebuilding after every code edit; you can omit it for production setups.
3. After the build finishes, source the workspace so ROS 2 can find the new executables:
   ```bash
   source install/setup.bash
   ```
4. Launch the complete stack. This starts joystick handling, kinematics, and the motor driver in one command:
   ```bash
   ros2 launch robot_launch robot_launch.launch.py
   ```

## Package Overview

### teleop_core
- Executable: `teleop_node`
- Subscribes to: `joy`
- Publishes: `/r1/cmd_vel`, `/r1/enable`

### kinematic_core
- Executable: `kinematic_node`
- Subscribes to: `/r1/cmd_vel`
- Publishes: `/r1/wheel_vel`

### roboteq_core
- Executable: `roboteq_node`
- Subscribes to: `/r1/wheel_vel`
- Streams wheel commands over serial to a Roboteq controller.

## Development Notes

- Launch files are installed via each package's `setup.py`.
- Adjust joystick axis and button mappings in `teleop_core/teleop_core/run_joy.py` if your controller differs.
