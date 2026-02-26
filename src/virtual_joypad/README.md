# virtual_joypad

## Overview
`virtual_joypad` provides PyQt6-based virtual controllers for robot teleoperation. It lets you send control commands from a GUI without physical joystick hardware, making it useful for development and testing. The package contains three nodes.

## Node: `virtual_controller`
Implementation: [`virtual_joypad/virtual_joypad/virtual_controller.py`](virtual_joypad/virtual_controller.py)

A tab-based arm controller that switches between Joint mode and Cartesian mode.

Key behaviours:
- Joint mode: control 1–12 joint angles via sliders (default ±90°).
- Cartesian mode: control position (x, y, z) and orientation (roll, pitch, yaw) via sliders.
- An Enable button starts and stops command publishing. Safety features such as automatic disable on tab switch are built in.
- Publishes: `/joint_cmd` (`Float64MultiArray`), `/cart_cmd` (`Twist`), `/joint_en`, `/cart_en`, `/enable` (`Bool`).
- Topic names, joint count, angle ranges, and unit conversion are configurable in `config/controller_config.yaml`.

## Node: `virtual_joy`
Implementation: [`virtual_joypad/virtual_joypad/virtual_joy.py`](virtual_joypad/virtual_joy.py)

An Xbox-style controller GUI rendered with QPainter. Analog sticks are draggable and their positions are retained after mouse release.

Key behaviours:
- Draggable analog sticks (Left/Right) and triggers (LT/RT).
- Toggleable buttons: A/B/X/Y, LB/RB, Back/Start/Xbox, L3/R3, D-Pad.
- Publishes: `/joy` (`sensor_msgs/Joy`). 8 axes, 11 buttons.

## Node: `virtual_joy_slider`
Implementation: [`virtual_joypad/virtual_joypad/virtual_joy_slider.py`](virtual_joypad/virtual_joy_slider.py)

A simpler, QSlider-based alternative to `virtual_joy`.

Key behaviours:
- Each axis (Left X/Y, Right X/Y, LT, RT) is controlled by a QSlider.
- Toggleable buttons and D-Pad share the same layout as `virtual_joy`.
- Publishes: `/joy` (`sensor_msgs/Joy`).

## Usage Tips
- PyQt6 is required: `pip install PyQt6`.
- Enable X11 forwarding if you are connecting over SSH.
- Edit `config/controller_config.yaml` to customise `virtual_controller` settings.

## Launch
Run `ros2 launch virtual_joypad virtual_controller.launch.py` to start `virtual_controller`. Each node can also be started directly with `ros2 run virtual_joypad <node>`.
