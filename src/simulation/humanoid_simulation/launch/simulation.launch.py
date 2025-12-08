#!/usr/bin/env python3
"""Launch Gazebo simulation with the humanoid robot."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Gazebo world file'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo with GUI'
        )
    )

    # Initialize arguments
    world_file = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    # Get paths
    pkg_humanoid_simulation = get_package_share_directory('humanoid_simulation')
    world_path = os.path.join(pkg_humanoid_simulation, 'worlds', 'empty.sdf')

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('humanoid_description'),
            'urdf',
            'humanoid.urdf.xacro'
        ]),
        ' use_sim:=true',
        ' use_fake_hardware:=false',
        ' use_gazebo:=true'
    ])

    robot_description = {'robot_description': robot_description_content}

    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_path],
        output='screen'
    )

    # Gazebo client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=IfCondition(gui)
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Bridge for clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Include controllers
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_controllers'),
                'launch',
                'controllers.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim': 'true',
            'use_fake_hardware': 'false',
        }.items()
    )

    nodes = [
        gazebo_server,
        gazebo_client,
        spawn_robot,
        robot_state_publisher,
        clock_bridge,
        controllers_launch,
    ]

    return LaunchDescription(declared_arguments + nodes)
