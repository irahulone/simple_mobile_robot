# robot_description

## Overview
`robot_description` contains the URDF/Xacro model, mesh, RViz configuration, and launch files for the rover robot. It provides the visual representation used by `robot_state_publisher` and RViz2.

## Contents
- `src/description/rover_robot.xacro` — URDF with prismatic x, y and revolute theta joints
- `meshes/rover_cad.stl` — 3-D mesh for RViz2 visualisation
- `rviz/rover.rviz` — RViz2 display configuration
- `launch/rover.launch.py` — starts `robot_state_publisher` with URDF and a static TF from `world` → `r1/world`
- `launch/display.launch.py` — starts RViz2 with the rover config

## TF Frame Tree
```
world
└── r1/world
    └── r1/x_axis
        └── r1/y_axis
            └── r1/theta_axis
                └── r1/base_link
                    └── r1/rover_link
```

## Launch / Usage
Typically launched as part of the simulation stack via `ros2 launch robot_launch sim.launch.py`. Can also be launched individually:
```bash
ros2 launch robot_description rover.launch.py
ros2 launch robot_description display.launch.py
```
