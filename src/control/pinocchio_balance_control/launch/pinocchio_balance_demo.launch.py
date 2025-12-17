#!/usr/bin/env python3
"""
Launch MuJoCo simulation with Pinocchio-based balance control.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import math


def generate_launch_description():
    # Launch arguments
    use_viewer_arg = DeclareLaunchArgument(
        'use_viewer',
        default_value='true',
        description='Whether to launch MuJoCo viewer'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('humanoid_description'),
            'urdf',
            'robot.xml'
        ]),
        description='Path to MuJoCo model file'
    )

    auto_step_arg = DeclareLaunchArgument(
        'auto_step',
        default_value='true',
        description='Start a few demo steps automatically'
    )

    step_count_arg = DeclareLaunchArgument(
        'step_count',
        default_value='4',
        description='How many steps to take when auto stepping'
    )

    step_pitch_amp_arg = DeclareLaunchArgument(
        'step_pitch_amp',
        default_value='0.24',
        description='Hip pitch amplitude for each step (rad)'
    )

    step_knee_lift_arg = DeclareLaunchArgument(
        'step_knee_lift',
        default_value='0.28',
        description='Knee lift amplitude for each step (rad)'
    )

    hip_roll_shift_arg = DeclareLaunchArgument(
        'hip_roll_shift',
        default_value='0.07',
        description='Lateral hip roll shift (rad)'
    )

    # MuJoCo Simulator Node
    mujoco_simulator = Node(
        package='humanoid_mujoco',
        executable='mujoco_simulator.py',
        name='mujoco_simulator',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'use_viewer': LaunchConfiguration('use_viewer'),
            'publish_rate': 100.0,
            'realtime_factor': 1.0,
            'use_sim_time': True
        }]
    )

    # Simple Position Controller (converts target_positions to joint_commands)
    simple_controller = Node(
        package='humanoid_mujoco',
        executable='mujoco_controller.py',
        name='mujoco_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Pinocchio Balance Controller
    pinocchio_controller = Node(
        package='pinocchio_balance_control',
        executable='pinocchio_balance_controller.py',
        name='pinocchio_balance_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'urdf_path': LaunchConfiguration('model_path'),
            'control_rate': 100.0,
            # Basic control gains
            'balance_kp': 5.0,              # Proportional gain
            'balance_kd': 0.5,              # Derivative gain (damping)
            'ankle_limit': 0.12,            # Max ankle correction (rad)
            'hip_limit': 0.15,              # Max hip correction (rad)
            # Stance configuration (natural posture)
            'knee_bend': 0.08,              # Reduced for more natural stance
            'stance_width': 0.06,           # Slightly narrower for stability
            # Advanced features
            'com_deadzone': 0.05,          # Ignore errors < 5mm (reduces oscillations)
            'filter_alpha': 0.4,            # Command smoothing (0=smooth, 1=responsive)
            'prediction_time': 0.2,         # Look-ahead time for anticipation (s)
            'auto_tune': False,             # Enable automatic gain tuning
            # Walking demo
            'auto_step': LaunchConfiguration('auto_step'),
            'step_count': LaunchConfiguration('step_count'),
            # Tuned defaults for stability (override-able)
            'step_pitch_amp': LaunchConfiguration('step_pitch_amp'),
            'step_knee_lift': LaunchConfiguration('step_knee_lift'),
            'hip_roll_shift': LaunchConfiguration('hip_roll_shift'),
            'walk_abort_margin': 1.5,
        }]
    )

    # Static transform publisher for world frame
    # This creates a 'world' frame at the origin for visualization
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        parameters=[{'use_sim_time': True}]
    )

    # RViz (optional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('humanoid_mujoco'),
        'config',
        'mujoco_visualization.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        use_viewer_arg,
        use_rviz_arg,
        model_path_arg,
        auto_step_arg,
        step_count_arg,
        step_pitch_amp_arg,
        step_knee_lift_arg,
        hip_roll_shift_arg,
        mujoco_simulator,
        simple_controller,
        pinocchio_controller,
        static_tf_world,  # Add world frame
        rviz,
    ])
