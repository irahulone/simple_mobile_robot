"""Launch robot state publisher with URDF and static TF for the rover."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
import launch_ros
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context)
    mesh_file = LaunchConfiguration('mesh_file').perform(context)

    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='robot_description'
    ).find('robot_description')
    xacro_file = os.path.join(pkg_share, 'description', 'rover_robot.xacro')

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='state_publisher',
            namespace=robot_id,
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ', xacro_file,
                    ' r:=1.0 g:=0.0 b:=0.0 a:=1.0',
                    f' mesh_file:={mesh_file}',
                ]),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_prefix': f'{robot_id}/',
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', f'{robot_id}/world'],
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id', default_value='r1',
            description='Robot identifier used as namespace and frame prefix',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Flag to enable use_sim_time',
        ),
        DeclareLaunchArgument(
            'mesh_file', default_value='rover_cad.stl',
            description='STL mesh filename',
        ),
        OpaqueFunction(function=launch_setup),
    ])
