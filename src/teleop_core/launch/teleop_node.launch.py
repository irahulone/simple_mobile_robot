from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Nodes
    run_joy_node = Node(
        package="joy",
        executable="joy_node",
    )
    joy_to_cmd_vel = Node(
        package="teleop_core",
        executable="teleop_node",
    )

    ld.add_action(run_joy_node)
    ld.add_action(joy_to_cmd_vel)

    return ld

