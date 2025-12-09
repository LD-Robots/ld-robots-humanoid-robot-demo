cmake_minimum_required(VERSION 3.8)
project(balance_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(controller_interface REQUIRED)
find_package(humanoid_msgs REQUIRED)
find_package(humanoid_utils REQUIRED)

# Include directories
include_directories(include)

# Dependencies list for linking
set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
  kdl_parser
  orocos_kdl
  controller_interface
  humanoid_msgs
  humanoid_utils
)

# IMU Feedback Processor Node
add_library(imu_feedback_processor SHARED
  src/imu_feedback_processor.cpp
)
target_compile_features(imu_feedback_processor PUBLIC cxx_std_17)
ament_target_dependencies(imu_feedback_processor ${dependencies})
rclcpp_components_register_nodes(imu_feedback_processor "balance_control::IMUFeedbackProcessor")

# CoM Controller Node
add_library(com_controller SHARED
  src/com_controller.cpp
)
target_compile_features(com_controller PUBLIC cxx_std_17)
ament_target_dependencies(com_controller ${dependencies})
rclcpp_components_register_nodes(com_controller "balance_control::CoMController")

# ZMP Stabilizer Node
add_library(zmp_stabilizer SHARED
  src/zmp_stabilizer.cpp
)
target_compile_features(zmp_stabilizer PUBLIC cxx_std_17)
ament_target_dependencies(zmp_stabilizer ${dependencies})
rclcpp_components_register_nodes(zmp_stabilizer "balance_control::ZMPStabilizer")

# Leg Balance Controller Node
add_library(leg_balance_controller SHARED
  src/leg_balance_controller.cpp
)
target_compile_features(leg_balance_controller PUBLIC cxx_std_17)
ament_target_dependencies(leg_balance_controller ${dependencies})
rclcpp_components_register_nodes(leg_balance_controller "balance_control::LegBalanceController")

# Create a component_container executable for running all nodes
find_package(rclcpp_components REQUIRED)
add_executable(balance_control_node src/balance_control_node.cpp)
target_compile_features(balance_control_node PUBLIC cxx_std_17)
target_link_libraries(balance_control_node
  imu_feedback_processor
  com_controller
  zmp_stabilizer
  leg_balance_controller
)
ament_target_dependencies(balance_control_node ${dependencies})

# Install libraries
install(TARGETS
  imu_feedback_processor
  com_controller
  zmp_stabilizer
  leg_balance_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  balance_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  imu_feedback_processor
  com_controller
  zmp_stabilizer
  leg_balance_controller
)
ament_export_dependencies(${dependencies})

ament_package()
