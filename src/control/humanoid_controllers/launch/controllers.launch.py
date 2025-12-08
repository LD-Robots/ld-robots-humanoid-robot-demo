#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use simulation (Gazebo) controllers'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware for testing'
        )
    )

    # Initialize arguments
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('humanoid_description'),
            'urdf',
            'humanoid.urdf.xacro'
        ]),
        ' use_sim:=',
        use_sim,
        ' use_fake_hardware:=',
        use_fake_hardware,
        ' use_gazebo:=false'
    ])

    robot_description = {'robot_description': robot_description_content}

    # Controller parameters
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('humanoid_controllers'),
        'config',
        'controllers.yaml'
    ])

    # Control node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
        remappings=[
            ('~/robot_description', '/robot_description'),
        ]
    )

    # Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    # IMU Sensor Broadcaster Spawner
    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager']
    )

    # Head Controller Spawner
    head_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['head_controller', '--controller-manager', '/controller_manager']
    )

    # Delay head controller after joint state broadcaster
    delay_head_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[head_controller_spawner],
        )
    )

    # Torso Controller Spawner
    torso_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['torso_controller', '--controller-manager', '/controller_manager']
    )

    delay_torso_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=head_controller_spawner,
            on_exit=[torso_controller_spawner],
        )
    )

    # Left Arm Controller Spawner
    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', '--controller-manager', '/controller_manager']
    )

    delay_left_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=torso_controller_spawner,
            on_exit=[left_arm_controller_spawner],
        )
    )

    # Right Arm Controller Spawner
    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', '--controller-manager', '/controller_manager']
    )

    delay_right_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )

    # Left Leg Controller Spawner
    left_leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_leg_controller', '--controller-manager', '/controller_manager']
    )

    delay_left_leg_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[left_leg_controller_spawner],
        )
    )

    # Right Leg Controller Spawner
    right_leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_leg_controller', '--controller-manager', '/controller_manager']
    )

    delay_right_leg_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_leg_controller_spawner,
            on_exit=[right_leg_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        delay_head_controller_spawner,
        delay_torso_controller_spawner,
        delay_left_arm_controller_spawner,
        delay_right_arm_controller_spawner,
        delay_left_leg_controller_spawner,
        delay_right_leg_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
