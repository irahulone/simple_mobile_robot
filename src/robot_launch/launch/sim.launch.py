"""Simulation preset: starts all nodes needed for a simulated environment."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_launch = os.path.join(
        get_package_share_directory('robot_launch'),
        'launch', 'bringup.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='r1',
                              description='Robot identifier'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'robot_id': LaunchConfiguration('robot_id'),
                'use_joy': 'true',
                'use_teleop': 'true',
                'use_kinematic': 'true',
                'use_sim': 'true',
                'use_description': 'true',
                'use_rviz': 'true',
            }.items(),
        ),
    ])
