#!/usr/bin/env python3
"""
Launch MuJoCo viewer with simple humanoid model.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get model path - use absolute path to source directory
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
        print(f"ERROR: No model found! Looked for:")
        print(f"  - {robot_mjcf}")
        print(f"  - {simple_model}")

    # Launch MuJoCo viewer
    mujoco_viewer = ExecuteProcess(
        cmd=['python3', '-m', 'mujoco.viewer', '--mjcf=' + model_path],
        output='screen',
        name='mujoco_viewer'
    )

    return LaunchDescription([
        mujoco_viewer
    ])
