#!/usr/bin/env python3
"""Launch file for perception nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Camera processor node
    camera_processor = Node(
        package='humanoid_perception',
        executable='camera_processor.py',
        name='camera_processor',
        output='screen',
        parameters=[{
            'camera_topic': '/camera/image_raw',
            'output_topic': '/camera/image_processed',
            'enable_visualization': True
        }]
    )

    # IMU filter node
    imu_filter = Node(
        package='humanoid_perception',
        executable='imu_filter.py',
        name='imu_filter',
        output='screen',
        parameters=[{
            'imu_topic': '/imu/data',
            'filtered_topic': '/imu/data_filtered',
            'filter_alpha': 0.9
        }]
    )

    return LaunchDescription([
        camera_processor,
        imu_filter,
    ])
