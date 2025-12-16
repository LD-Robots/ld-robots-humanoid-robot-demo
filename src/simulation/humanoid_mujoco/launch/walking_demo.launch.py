#!/usr/bin/env python3
"""
Launch MuJoCo simulation with walking controller.
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

    gait_type_arg = DeclareLaunchArgument(
        'gait_type',
        default_value='walk',
        description='Type of gait (walk, run, etc.)'
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

    # Locomotion Controller (when implemented)
    locomotion_controller = Node(
        package='locomotion_control',
        executable='locomotion_controller',
        name='locomotion_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'control_rate': 100.0,
            'gait_type': LaunchConfiguration('gait_type'),
        }]
    )

    # Footstep Planner (when implemented)
    footstep_planner = Node(
        package='locomotion_planning',
        executable='footstep_planner',
        name='footstep_planner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        use_viewer_arg,
        gait_type_arg,
        simulation_launch,
        # locomotion_controller,  # Uncomment when locomotion_control package is ready
        # footstep_planner,  # Uncomment when locomotion_planning package is ready
    ])
