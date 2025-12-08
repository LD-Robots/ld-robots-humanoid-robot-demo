#!/usr/bin/env python3
"""Launch file for testing with fake hardware (no physical robot needed)."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include the controllers launch file with fake hardware settings
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_controllers'),
                'launch',
                'controllers.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim': 'false',
            'use_fake_hardware': 'true',
        }.items()
    )

    return LaunchDescription([
        controllers_launch,
    ])
