#!/usr/bin/env python3
"""
Launch walking controller with MuJoCo simulation.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get model path
    workspace_dir = os.path.expanduser('~/ros2_ws_demo/ld-robots-humanoid-robot-demo')

    # Priority: robot.mjcf (real robot) > simple geometric model
    robot_mjcf = os.path.join(workspace_dir, 'src/control/lipm_walking_controller/models/robot.mjcf')
    simple_model = os.path.join(workspace_dir, 'src/control/lipm_walking_controller/models/simple_humanoid.xml')

    if os.path.exists(robot_mjcf):
        model_path = robot_mjcf
        print(f"✓ Using REAL robot model with meshes: {model_path}")
    elif os.path.exists(simple_model):
        model_path = simple_model
        print(f"✓ Using simple geometric model: {model_path}")
    else:
        model_path = simple_model
        print(f"ERROR: No model found! Using: {model_path}")

    # Get walking controller script path
    controller_script = os.path.join(workspace_dir, 'src/control/lipm_walking_controller/scripts/test_mujoco_walking.py')

    # Launch walking controller
    walking_controller = ExecuteProcess(
        cmd=['python3', controller_script, model_path, '4'],  # 4 steps
        output='screen',
        name='walking_controller',
        cwd=workspace_dir  # Set working directory
    )

    return LaunchDescription([
        walking_controller
    ])
