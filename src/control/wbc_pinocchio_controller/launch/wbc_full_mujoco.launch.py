#!/usr/bin/env python3
"""Launch MuJoCo and WBC controller with startup delay and namespace support."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Namespace argument for parallel robot isolation
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes (empty = no namespace, e.g., robot_0)'
    )

    delay_arg = DeclareLaunchArgument(
        'controller_delay',
        default_value='0.0',
        description='Seconds to delay WBC controller start'
    )
    walking_enabled_arg = DeclareLaunchArgument(
        'walking_enabled',
        default_value='true',
        description='Enable WBC walking phases'
    )

    use_viewer_arg = DeclareLaunchArgument(
        'use_viewer',
        default_value='true',
        description='Whether to launch MuJoCo viewer'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    wbc_config_arg = DeclareLaunchArgument(
        'wbc_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('wbc_pinocchio_controller'),
            'config',
            'wbc_controller.yaml'
        ]),
        description='WBC controller YAML config path'
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
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Rate at which to publish robot state (Hz)'
    )
    publish_clock_arg = DeclareLaunchArgument(
        'publish_clock',
        default_value='true',
        description='Whether to publish /clock from this simulation'
    )
    realtime_factor_arg = DeclareLaunchArgument(
        'realtime_factor',
        default_value='1.0',
        description='Simulation speed (1.0 = realtime)'
    )
    torso_body_name_arg = DeclareLaunchArgument(
        'torso_body_name',
        default_value='torso',
        description='Body name used for torso TF and torso pose'
    )
    pelvis_body_name_arg = DeclareLaunchArgument(
        'pelvis_body_name',
        default_value='pelvis',
        description='Body name used for pelvis TF'
    )
    hold_start_duration_arg = DeclareLaunchArgument(
        'hold_start_duration',
        default_value='5.0',
        description='Seconds to hold initial joint pose before accepting commands'
    )
    initial_pose_yaml_arg = DeclareLaunchArgument(
        'initial_pose_yaml',
        default_value=PathJoinSubstitution([
            FindPackageShare('wbc_pinocchio_controller'),
            'config',
            'initial_pose.yaml'
        ]),
        description='YAML file with initial joint_positions'
    )
    initial_pose_key_arg = DeclareLaunchArgument(
        'initial_pose_key',
        default_value='',
        description='Top-level key in YAML to use for joint_positions'
    )

    mujoco_simulator = Node(
        package='humanoid_mujoco',
        executable='mujoco_simulator.py',
        name='mujoco_simulator',
        output='screen',
        remappings=[
            ('clock', '/clock'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'use_viewer': LaunchConfiguration('use_viewer'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'realtime_factor': LaunchConfiguration('realtime_factor'),
            'publish_clock': LaunchConfiguration('publish_clock'),
            'torso_body_name': LaunchConfiguration('torso_body_name'),
            'pelvis_body_name': LaunchConfiguration('pelvis_body_name'),
            'hold_start_duration': LaunchConfiguration('hold_start_duration'),
            'initial_pose_yaml': LaunchConfiguration('initial_pose_yaml'),
            'initial_pose_key': LaunchConfiguration('initial_pose_key'),
            'use_sim_time': True
        }]
    )

    params_file = LaunchConfiguration('wbc_config')

    controller_node = Node(
        package='wbc_pinocchio_controller',
        executable='wbc_controller',
        name='wbc_controller',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': True,
                'walking_enabled': LaunchConfiguration('walking_enabled')
            }
        ],
        remappings=[
            # Remap all topics to namespaced versions
            ('/target_positions', 'target_positions'),
            ('/joint_states', 'joint_states'),
            ('/imu/data', 'imu/data'),
            ('/torso/pose', 'torso/pose'),
            ('/pelvis/pose', 'pelvis/pose'),
        ]
    )
    mujoco_controller = Node(
        package='humanoid_mujoco',
        executable='mujoco_controller.py',
        name='mujoco_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'initial_pose_yaml': LaunchConfiguration('initial_pose_yaml'),
            'initial_pose_key': LaunchConfiguration('initial_pose_key'),
        }]
    )

    controller_delayed = TimerAction(
        period=LaunchConfiguration('controller_delay'),
        actions=[controller_node]
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('humanoid_mujoco'),
        'config',
        'mujoco_visualization.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': True}]
    )

    # Group all nodes with namespace
    namespaced_nodes = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            mujoco_simulator,
            mujoco_controller,
            controller_delayed,
        ]
    )

    return LaunchDescription([
        namespace_arg,
        delay_arg,
        walking_enabled_arg,
        use_viewer_arg,
        use_rviz_arg,
        wbc_config_arg,
        model_path_arg,
        publish_rate_arg,
        publish_clock_arg,
        realtime_factor_arg,
        torso_body_name_arg,
        pelvis_body_name_arg,
        hold_start_duration_arg,
        initial_pose_yaml_arg,
        initial_pose_key_arg,
        namespaced_nodes,
        rviz_node,  # RViz stays in root namespace
    ])
