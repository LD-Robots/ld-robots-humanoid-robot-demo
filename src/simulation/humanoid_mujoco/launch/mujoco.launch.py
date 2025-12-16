#!/usr/bin/env python3
"""
Launch file to visualize humanoid robot in MuJoCo
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_humanoid_mujoco = FindPackageShare('humanoid_mujoco')
    
    mjcf_path = PathJoinSubstitution([
        pkg_humanoid_mujoco,
        'mujoco',
        'humanoid.xml'
    ])

    # Command to run MuJoCo viewer
    mujoco_viewer_cmd = ExecuteProcess(
        cmd=['python3', '-m', 'mujoco.viewer', '--mjcf', mjcf_path],
        output='screen'
    )

    return LaunchDescription([
        mujoco_viewer_cmd
    ])
