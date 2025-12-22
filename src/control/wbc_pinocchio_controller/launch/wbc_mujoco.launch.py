#!/usr/bin/env python3
"""Launch WBC Pinocchio controller for MuJoCo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('wbc_pinocchio_controller'),
        'config',
        'wbc_controller.yaml'
    ])

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    controller_node = Node(
        package='wbc_pinocchio_controller',
        executable='wbc_controller',
        name='wbc_controller',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        controller_node,
    ])
