#!/usr/bin/env python3
"""
Launch file for Forward Walking Controller on robot.mjcf
Pași înainte (forward steps), optimizat pentru URDF local
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    """Generate launch description"""

    workspace_dir = os.path.expanduser('~/ros2_ws_demo/ld-robots-humanoid-robot-demo')

    # Use robot.mjcf (URDF local)
    model_path = os.path.join(
        workspace_dir,
        'src/control/lipm_walking_controller/models/robot.mjcf'
    )

    controller_script = os.path.join(
        workspace_dir,
        'src/control/lipm_walking_controller/scripts/forward_walking_robot.py'
    )

    os.chmod(controller_script, 0o755)

    # Launch forward walking controller
    walking_controller = ExecuteProcess(
        cmd=['python3', '-u', controller_script, model_path, '50'],
        output='screen',
        name='forward_walking_robot',
        cwd=workspace_dir,
        emulate_tty=True,
    )

    return LaunchDescription([
        walking_controller
    ])
