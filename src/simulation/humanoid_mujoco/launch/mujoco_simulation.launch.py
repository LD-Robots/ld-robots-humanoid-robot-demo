#!/usr/bin/env python3
"""
Launch MuJoCo simulation for humanoid robot.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    humanoid_mujoco_dir = get_package_share_directory('humanoid_mujoco')
    humanoid_description_dir = get_package_share_directory('humanoid_description')

    # Launch arguments
    use_viewer_arg = DeclareLaunchArgument(
        'use_viewer',
        default_value='true',
        description='Whether to launch MuJoCo viewer'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('humanoid_mujoco'),
            'models',
            'humanoid.xml'
        ]),
        description='Path to MuJoCo model file'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Rate at which to publish robot state (Hz)'
    )

    realtime_factor_arg = DeclareLaunchArgument(
        'realtime_factor',
        default_value='1.0',
        description='Simulation speed (1.0 = realtime)'
    )

    # URDF for RViz visualization
    urdf_path = PathJoinSubstitution([
        FindPackageShare('humanoid_description'),
        'urdf',
        'humanoid.urdf.xacro'
    ])

    robot_description = Command(['xacro ', urdf_path])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # MuJoCo Simulator Node
    mujoco_simulator = Node(
        package='humanoid_mujoco',
        executable='mujoco_simulator.py',
        name='mujoco_simulator',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'use_viewer': LaunchConfiguration('use_viewer'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'realtime_factor': LaunchConfiguration('realtime_factor'),
            'use_sim_time': True
        }]
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('humanoid_mujoco'),
        'config',
        'mujoco_visualization.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        use_viewer_arg,
        use_rviz_arg,
        model_path_arg,
        publish_rate_arg,
        realtime_factor_arg,
        robot_state_publisher,
        mujoco_simulator,
        rviz,
    ])
