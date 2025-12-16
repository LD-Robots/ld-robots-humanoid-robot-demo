cmake_minimum_required(VERSION 3.8)
project(humanoid_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(humanoid_msgs REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ros2_control REQUIRED)

# Include directories
include_directories(
  include
)

# MuJoCo ROS 2 Bridge Node
add_executable(mujoco_ros_bridge
  src/mujoco_ros_bridge.cpp
)

ament_target_dependencies(mujoco_ros_bridge
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  humanoid_msgs
  controller_manager
  hardware_interface
)

# Install C++ executables
install(TARGETS
  mujoco_ros_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/mujoco_simulator.py
  scripts/mujoco_visualizer.py
  scripts/urdf_to_mjcf.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
