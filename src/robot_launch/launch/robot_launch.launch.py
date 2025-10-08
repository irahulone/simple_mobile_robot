from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_core',
            executable='teleop_node',
            output='screen',
        ),
        Node(
            package='kinematic_core',
            executable='kinematic_node',
            output='screen',
        ),
        Node(
            package='roboteq_core',
            executable='roboteq_node',
            output='screen',
        ),
    ])
