#!/usr/bin/env python3
"""
Launch MuJoCo simulation for humanoid robot.
Uses the existing robot.xml from humanoid_description package.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_viewer_arg = DeclareLaunchArgument(
        'use_viewer',
        default_value='true',
        description='Whether to launch MuJoCo viewer'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('humanoid_description'),
            'urdf',
            'robot.xml'
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

    torso_body_name_arg = DeclareLaunchArgument(
        'torso_body_name',
        default_value='torso',
        description='Body name used for torso TF and torso pose'
    )

    pelvis_body_name_arg = DeclareLaunchArgument(
        'pelvis_body_name',
        default_value='pelvis',
        description='Body name used for pelvis TF'
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
            'torso_body_name': LaunchConfiguration('torso_body_name'),
            'pelvis_body_name': LaunchConfiguration('pelvis_body_name'),
            'use_sim_time': True
        }]
    )

    # RViz (optional)
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
        torso_body_name_arg,
        pelvis_body_name_arg,
        mujoco_simulator,
        rviz,
    ])
