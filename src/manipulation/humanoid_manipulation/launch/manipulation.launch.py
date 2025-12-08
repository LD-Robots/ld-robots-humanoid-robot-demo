#!/usr/bin/env python3
"""Launch file for manipulation nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Left gripper controller
    left_gripper_controller = Node(
        package='humanoid_manipulation',
        executable='gripper_controller.py',
        name='left_gripper_controller',
        output='screen',
        parameters=[{
            'gripper_side': 'left',
            'max_position': 0.04,
            'min_position': 0.0
        }]
    )

    # Right gripper controller
    right_gripper_controller = Node(
        package='humanoid_manipulation',
        executable='gripper_controller.py',
        name='right_gripper_controller',
        output='screen',
        parameters=[{
            'gripper_side': 'right',
            'max_position': 0.04,
            'min_position': 0.0
        }]
    )

    # Left arm motion planner
    left_arm_planner = Node(
        package='humanoid_manipulation',
        executable='arm_motion_planner.py',
        name='left_arm_motion_planner',
        output='screen',
        parameters=[{
            'arm_side': 'left'
        }]
    )

    # Right arm motion planner
    right_arm_planner = Node(
        package='humanoid_manipulation',
        executable='arm_motion_planner.py',
        name='right_arm_motion_planner',
        output='screen',
        parameters=[{
            'arm_side': 'right'
        }]
    )

    return LaunchDescription([
        left_gripper_controller,
        right_gripper_controller,
        left_arm_planner,
        right_arm_planner,
    ])
