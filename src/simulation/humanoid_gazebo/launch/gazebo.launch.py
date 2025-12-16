#!/usr/bin/env python3
"""
Launch Gazebo simulation with the humanoid robot.

This launch file:
1. Starts Gazebo Harmonic with a world file
2. Spawns the humanoid robot from humanoid_leg_description
3. Sets up the ROS-Gazebo bridge for topics
4. Configures robot_state_publisher
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='World file name (in worlds/ directory)'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo GUI'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode (no GUI)'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use simulation hardware interface'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix for robot links and joints'
        )
    )

    # Initialize Arguments
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    use_sim = LaunchConfiguration('use_sim')
    prefix = LaunchConfiguration('prefix')

    # Get package install directories
    install_dir = get_package_prefix('humanoid_description')
    gazebo_install_dir = get_package_prefix('humanoid_gazebo')

    # Set GZ_SIM_RESOURCE_PATH to ROS workspace for package:// URI resolution
    # Include both humanoid_description and humanoid_gazebo (for pressure mat model)
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(install_dir, 'share') + ':' + os.path.join(gazebo_install_dir, 'share/humanoid_gazebo/models')
    )

    # Set GZ_SIM_SYSTEM_PLUGIN_PATH to find gz_ros2_control plugin
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/jazzy/lib'
    )

    # Paths
    world_file = PathJoinSubstitution([
        FindPackageShare('humanoid_gazebo'),
        'worlds',
        world
    ])

    urdf_file = PathJoinSubstitution([
        FindPackageShare('humanoid_description'),
        'urdf',
        'humanoid.urdf.xacro'
    ])

    controllers_file = PathJoinSubstitution([
        FindPackageShare('humanoid_gazebo'),
        'config',
        'controllers.yaml'
    ])

    # Get URDF via xacro
    humanoid_leg_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            urdf_file,
            ' use_sim:=',
            use_sim,
            ' prefix:=',
            prefix
        ]),
        value_type=str
    )

    robot_description = {'robot_description': humanoid_leg_description_content}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Gazebo simulation server (headless)
    # Using gz_server.launch.py for server-only mode
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_ros_gz_sim,
                "launch",
                "gz_server.launch.py"  # Server-only, no GUI
            ])
        ),
        launch_arguments={
            "world_sdf_file": world_file,
            "gz_args": "-r -v 4",  # -r auto-start, -v 4 verbose
        }.items()
    )

    # Spawn robot
    # Z position calculated as: leg_length (0.749) + foot_collision_offset (0.051) - penetration (0.005) = 0.795
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'kbot_humanoid',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.2',  # Position robot so feet make contact with ground
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge for clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge for IMU sensor
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge for left foot force-torque sensor
    left_ft_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/left_foot/ft_data@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge for right foot force-torque sensor
    right_ft_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/right_foot/ft_data@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge for left foot contact sensor
    # Format: topic@ros_type]gz_type (GZ->ROS direction uses ])
    left_contact_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty_world/model/kbot_humanoid/link/LFootBushing_GPF_1517_12/sensor/left_contact_sensor/contact@ros_gz_interfaces/msg/Contacts]gz.msgs.Contacts',
            '--ros-args', '-r', '/world/empty_world/model/kbot_humanoid/link/LFootBushing_GPF_1517_12/sensor/left_contact_sensor/contact:=/left_foot/contact'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge for right foot contact sensor
    right_contact_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty_world/model/kbot_humanoid/link/RFootBushing_GPF_1517_12/sensor/right_contact_sensor/contact@ros_gz_interfaces/msg/Contacts]gz.msgs.Contacts',
            '--ros-args', '-r', '/world/empty_world/model/kbot_humanoid/link/RFootBushing_GPF_1517_12/sensor/right_contact_sensor/contact:=/right_foot/contact'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge for pressure mat contact sensor
    pressure_mat_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty_world/model/pressure_mat/link/mat_base/sensor/pressure_mat_contact_sensor/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'
        ],
        output='screen'
    )

    # Pressure Mat Visualizer Node
    pressure_visualizer = Node(
        package='humanoid_gazebo',
        executable='pressure_mat_visualizer.py',
        name='pressure_mat_visualizer',
        output='screen',
        parameters=[
            {'grid_size_x': 100},        # Very dense grid for detailed foot outline
            {'grid_size_y': 60},         # Very dense grid (5x more detail than before)
            {'mat_width': 1.0},
            {'mat_height': 0.6},
            {'update_rate': 30.0},
            {'force_scale': 10.0},       # Moderate sensitivity
            {'decay_rate': 0.1},         # Not used anymore (replaced by smoothing_alpha)
            {'smoothing_alpha': 0.2}     # Smooth temporal blending (20% new, 80% old)
        ]
    )

    # Include the controller spawner launch file
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('humanoid_control'),
                'launch',
                'control.launch.py'
            ])
        )
    )
    
    # Create launch description and populate
    ld = LaunchDescription(declared_arguments)

    # Add environment variables
    ld.add_action(gz_resource_path)
    ld.add_action(gz_plugin_path)

    # Add nodes to launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gazebo_sim)
    ld.add_action(spawn_robot)
    ld.add_action(clock_bridge)
    ld.add_action(imu_bridge)
    ld.add_action(left_ft_bridge)
    ld.add_action(right_ft_bridge)
    ld.add_action(left_contact_bridge)
    ld.add_action(right_contact_bridge)
    ld.add_action(pressure_mat_bridge)
    ld.add_action(pressure_visualizer)

    # Add controller spawners (with delay to wait for Gazebo)
    control_launch_delayed = TimerAction(
        period=5.0,
        actions=[control_launch]
    )
    ld.add_action(control_launch_delayed)
    return ld
