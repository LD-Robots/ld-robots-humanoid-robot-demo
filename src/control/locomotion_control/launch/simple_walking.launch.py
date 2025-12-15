#!/usr/bin/env python3
"""
Launch file for Simple Walking Controller (WORKING)
Uses simple_humanoid.xml which has proven stable walking
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    """Generate launch description for simple walking controller"""

    workspace_dir = os.path.expanduser('~/ros2_ws_demo/ld-robots-humanoid-robot-demo')

    # Use simple_humanoid.xml (STABLE, WORKING)
    model_path = os.path.join(
        workspace_dir,
        'src/control/lipm_walking_controller/models/simple_humanoid.xml'
    )

    controller_script = os.path.join(
        workspace_dir,
        'src/control/lipm_walking_controller/scripts/simple_walking_controller.py'
    )

    os.chmod(controller_script, 0o755)

    # Launch walking controller (30 seconds, 8 steps)
    walking_controller = ExecuteProcess(
        cmd=['python3', '-u', controller_script, model_path, '30'],
        output='screen',
        name='simple_walking_controller',
        cwd=workspace_dir,
        emulate_tty=True,
    )

    return LaunchDescription([
        walking_controller
    ])
