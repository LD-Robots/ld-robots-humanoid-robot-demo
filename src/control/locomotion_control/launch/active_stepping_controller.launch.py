#!/usr/bin/env python3
"""
Launch file for Active Stepping Walking Controller with MuJoCo
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    """Generate launch description for active stepping walking controller"""

    # Get paths
    workspace_dir = os.path.expanduser('~/ros2_ws_demo/ld-robots-humanoid-robot-demo')
    robot_mjcf = os.path.join(
        workspace_dir,
        'src/control/lipm_walking_controller/models/robot.mjcf'
    )
    controller_script = os.path.join(
        workspace_dir,
        'src/control/lipm_walking_controller/scripts/active_stepping_controller.py'
    )

    # Make script executable
    os.chmod(controller_script, 0o755)

    # Launch active stepping walking controller
    active_stepping_controller = ExecuteProcess(
        cmd=['python3', '-u', controller_script, robot_mjcf, '60'],  # 60 seconds
        output='screen',
        name='active_stepping_controller',
        cwd=workspace_dir,
        emulate_tty=True,
    )

    return LaunchDescription([
        active_stepping_controller
    ])
