"""Base launch file with per-node toggle flags.

Every node can be individually enabled or disabled via launch arguments.
Preset launch files (sim.launch.py, actual.launch.py) include this file
with appropriate defaults.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context)
    kinematic_type = LaunchConfiguration('kinematic_type').perform(context)

    config_file = os.path.join(
        get_package_share_directory('robot_launch'),
        'config', f'{robot_id}.yaml',
    )

    kinematic_exec = 'kinematic_node' if kinematic_type == 'diff' else 'omni_kinematic_node'
    mesh_file = 'rover_cad.stl' if kinematic_type == 'diff' else 'omni_cad.stl'

    desc_dir = get_package_share_directory('robot_description')
    rover_launch = os.path.join(desc_dir, 'launch', 'rover.launch.py')
    display_launch = os.path.join(desc_dir, 'launch', 'display.launch.py')

    actions = [
        # robot_description (URDF + TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rover_launch),
            launch_arguments={'robot_id': robot_id, 'mesh_file': mesh_file}.items(),
            condition=IfCondition(LaunchConfiguration('use_description')),
        ),

        # RViz2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch),
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),

        # virtual_joy
        Node(
            package='virtual_joypad',
            executable='virtual_joy',
            name='virtual_joy',
            namespace=robot_id,
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_joy')),
        ),

        # teleop_node
        Node(
            package='teleop_core',
            executable='teleop_node',
            name='teleop_node',
            namespace=robot_id,
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_teleop')),
        ),

        # kinematic_node (diff or omni based on kinematic_type)
        Node(
            package='kinematic_core',
            executable=kinematic_exec,
            name='kinematic_node',
            namespace=robot_id,
            output='screen',
            parameters=[config_file],
            condition=IfCondition(LaunchConfiguration('use_kinematic')),
        ),

        # sim_robot_node
        Node(
            package='sim_robot',
            executable='sim_robot_node',
            name='sim_robot_node',
            namespace=robot_id,
            output='screen',
            parameters=[config_file],
            condition=IfCondition(LaunchConfiguration('use_sim')),
        ),

        # roboteq_node
        Node(
            package='roboteq_core',
            executable='roboteq_node',
            name='roboteq_node',
            namespace=robot_id,
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_actual')),
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='r1',
                              description='Robot identifier (namespace + config selection)'),
        DeclareLaunchArgument('use_joy', default_value='false',
                              description='Launch virtual_joy node'),
        DeclareLaunchArgument('use_teleop', default_value='true',
                              description='Launch teleop_node'),
        DeclareLaunchArgument('use_kinematic', default_value='true',
                              description='Launch kinematic_node'),
        DeclareLaunchArgument('kinematic_type', default_value='diff',
                              description='Kinematic model: diff or omni'),
        DeclareLaunchArgument('use_sim', default_value='false',
                              description='Launch sim_robot_node'),
        DeclareLaunchArgument('use_actual', default_value='false',
                              description='Launch roboteq_node'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Launch RViz2'),
        DeclareLaunchArgument('use_description', default_value='false',
                              description='Launch robot_state_publisher + static TF'),
        OpaqueFunction(function=_launch_nodes),
    ])
