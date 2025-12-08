#!/usr/bin/env python3
"""Launch file for locomotion nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Gait generator node
    gait_generator = Node(
        package='humanoid_locomotion',
        executable='gait_generator.py',
        name='gait_generator',
        output='screen',
        parameters=[{
            'step_length': 0.1,
            'step_height': 0.05,
            'step_duration': 0.8,
            'double_support_ratio': 0.2
        }]
    )

    # Balance controller node
    balance_controller = Node(
        package='humanoid_locomotion',
        executable='balance_controller.py',
        name='balance_controller',
        output='screen',
        parameters=[{
            'kp_pitch': 0.5,
            'kp_roll': 0.5,
            'kd_pitch': 0.1,
            'kd_roll': 0.1
        }]
    )

    return LaunchDescription([
        gait_generator,
        balance_controller,
    ])
