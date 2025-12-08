#!/usr/bin/env python3
"""Launch the complete humanoid robot system with all capabilities."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Simulation
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_simulation'),
                'launch',
                'simulation.launch.py'
            ])
        ])
    )

    # Perception
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_perception'),
                'launch',
                'perception.launch.py'
            ])
        ])
    )

    # Locomotion
    locomotion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_locomotion'),
                'launch',
                'locomotion.launch.py'
            ])
        ])
    )

    # Manipulation
    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_manipulation'),
                'launch',
                'manipulation.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        simulation_launch,
        perception_launch,
        locomotion_launch,
        manipulation_launch,
    ])
