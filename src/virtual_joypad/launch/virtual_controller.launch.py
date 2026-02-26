"""Launch file for Virtual Robot Arm Controller."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for virtual controller."""

    # Get package directory
    pkg_dir = get_package_share_directory('virtual_joypad')

    # Config file path (optional, can be overridden)
    config_file = os.path.join(pkg_dir, 'config', 'controller_config.yaml')

    return LaunchDescription([
        Node(
            package='virtual_joypad',
            executable='virtual_controller',
            name='virtual_controller',
            output='screen',
            emulate_tty=True,
        )
    ])
