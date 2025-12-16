#!/usr/bin/env python3
"""
Launch MuJoCo simulation with balance controller.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    humanoid_mujoco_dir = get_package_share_directory('humanoid_mujoco')

    # Launch arguments
    use_viewer_arg = DeclareLaunchArgument(
        'use_viewer',
        default_value='true',
        description='Whether to launch MuJoCo viewer'
    )

    # Include main simulation launch
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_mujoco'),
                'launch',
                'mujoco_simulation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_viewer': LaunchConfiguration('use_viewer'),
            'use_rviz': 'true',
        }.items()
    )

    # Balance Controller (when implemented)
    balance_controller = Node(
        package='balance_control',
        executable='balance_controller',
        name='balance_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'control_rate': 100.0,
        }]
    )

    return LaunchDescription([
        use_viewer_arg,
        simulation_launch,
        # balance_controller,  # Uncomment when balance_control package is ready
    ])
