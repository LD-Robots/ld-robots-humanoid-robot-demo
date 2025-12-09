#!/usr/bin/env python3
"""
Balance Control System Launch File

This launch file starts all balance control nodes:
1. IMU Feedback Processor - Filters and processes IMU data
2. CoM Controller - Manages center of mass positioning
3. ZMP Stabilizer - Ensures ZMP stays within support polygon

All nodes are configured using the balance_params.yaml file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """
    Setup function to resolve launch configurations
    """
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    # Get package share directory
    balance_control_share = get_package_share_directory('balance_control')

    # Path to config file
    config_file = PathJoinSubstitution([
        FindPackageShare('balance_control'),
        'config',
        'balance_params.yaml'
    ])

    # Get robot description from robot_state_publisher if running
    # For now, we'll make robot_description optional in CoM controller
    # The controller can work with a simplified model

    # Load nodes as components in a component container for better performance
    from launch_ros.actions import ComposableNodeContainer
    from launch_ros.descriptions import ComposableNode

    container = ComposableNodeContainer(
        name='balance_control_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='balance_control',
                plugin='balance_control::IMUFeedbackProcessor',
                name='imu_feedback_processor',
                parameters=[
                    config_file,
                    {'use_sim_time': use_sim_time}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ('/imu/data', '/imu/data'),
                ]
            ),
            ComposableNode(
                package='balance_control',
                plugin='balance_control::CoMController',
                name='com_controller',
                parameters=[
                    config_file,
                    {'use_sim_time': use_sim_time}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ('/joint_states', '/joint_states'),
                    ('/robot_description', '/robot_description'),
                ]
            ),
            ComposableNode(
                package='balance_control',
                plugin='balance_control::ZMPStabilizer',
                name='zmp_stabilizer',
                parameters=[
                    config_file,
                    {'use_sim_time': use_sim_time}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='balance_control',
                plugin='balance_control::LegBalanceController',
                name='leg_balance_controller',
                parameters=[
                    config_file,
                    {'use_sim_time': use_sim_time}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ('/joint_states', '/joint_states'),
                    ('/robot_description', '/robot_description'),
                ]
            ),
        ],
        output='screen',
    )

    return [container]


def generate_launch_description():
    """
    Generate launch description for balance control system
    """
    # Declare launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (debug, info, warn, error, fatal)',
            choices=['debug', 'info', 'warn', 'error', 'fatal']
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('balance_control'),
                'config',
                'balance_params.yaml'
            ]),
            description='Path to balance control parameters file'
        )
    )

    # Create launch description
    ld = LaunchDescription(declared_arguments)

    # Add opaque function to handle launch configuration resolution
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
