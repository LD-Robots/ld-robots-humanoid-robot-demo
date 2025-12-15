#!/usr/bin/env python3
"""
Launch robot_balanced.mjcf with ZMP balance controller (standing test).
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get workspace directory
    workspace_dir = os.path.expanduser('~/ros2_ws_demo/ld-robots-humanoid-robot-demo')

    # robot_balanced.mjcf
    model_path = os.path.join(workspace_dir, 'src/control/lipm_walking_controller/models/robot_balanced.mjcf')

    # ZMP balance controller script
    controller_script = os.path.join(workspace_dir, 'src/control/lipm_walking_controller/scripts/zmp_balance_controller.py')

    # Declare launch arguments
    duration_arg = DeclareLaunchArgument(
        'duration',
        default_value='120.0',
        description='Simulation duration in seconds'
    )

    # Check if model exists
    if os.path.exists(model_path):
        print(f"✓ Using robot_balanced.mjcf: {model_path}")
    else:
        print(f"ERROR: robot_balanced.mjcf not found at {model_path}")

    # Launch ZMP balance controller
    balance_controller = ExecuteProcess(
        cmd=['python3', '-u', controller_script, model_path, LaunchConfiguration('duration')],
        output='screen',
        name='robot_balanced_standing_controller',
        cwd=workspace_dir
    )

    return LaunchDescription([
        duration_arg,
        balance_controller
    ])
