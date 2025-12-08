# Package Dependencies Status

## Successfully Built (21 packages)
✅ Core packages that build without external dependencies:

### Common
- humanoid_msgs
- humanoid_interfaces
- humanoid_utils

### Robot Description
- humanoid_description

### Hardware Interface
- humanoid_hardware
- actuator_interfaces
- sensor_interfaces
- safety_monitor
- diagnostics

### Control
- humanoid_control
- upper_body_control
- lower_body_control
- arm_control
- hand_control
- leg_control
- balance_control
- locomotion_control
- bimanual_control

### Localization (partial)
- slam
- localization

### Bringup
- humanoid_bringup

## Packages with Missing External Dependencies

### Behavior Packages (4 packages)
**Missing**: `behaviortree_cpp_v3`

- behavior_trees
- manipulation_behaviors
- locomotion_behaviors
- interaction_behaviors

**Install command**:
```bash
sudo apt install ros-jazzy-behaviortree-cpp-v3
```

### Localization Package (1 package)
**Missing**: `robot_localization`

- state_estimation

**Install command**:
```bash
sudo apt install ros-jazzy-robot-localization
```

### Navigation Packages (3 packages)
**Missing**: `nav2_common`, `nav2_msgs`

- humanoid_navigation
- footstep_planning
- obstacle_avoidance

**Install command**:
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-common
```

### Planning Packages (7 packages)
**Missing**: `moveit_ros_planning_interface`, `moveit_ros_move_group`, `moveit_task_constructor_core`

- humanoid_moveit_config
- upper_body_moveit_config
- lower_body_moveit_config
- manipulation_planning
- humanoid_mtc
- trajectory_optimization
- manipulation_library (in applications)

**Install command**:
```bash
sudo apt install ros-jazzy-moveit ros-jazzy-moveit-task-constructor-core
```

### Perception Packages (7 packages)
**Missing**: `cv_bridge`, `image_transport`, `vision_msgs`, `pcl_ros`, `octomap_msgs`

- vision_pipeline
- object_perception
- scene_understanding
- point_cloud_processing
- semantic_mapping
- visual_servoing
- human_detection

**Install command**:
```bash
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-vision-msgs ros-jazzy-pcl-ros ros-jazzy-octomap-msgs
```

### Simulation Packages (3 packages)
**Missing**: `gazebo_ros`, `gazebo_plugins`

- humanoid_gazebo
- isaac_sim
- simulation_tools

**Install command**:
```bash
sudo apt install ros-jazzy-gazebo-ros-pkgs
```

### Tools Packages (2 packages)
**Missing**: `rosbag2_cpp`, `rviz2`

- recording_tools
- visualization_tools

**Install command**:
```bash
sudo apt install ros-jazzy-rosbag2 ros-jazzy-rviz2
```

## Install All Dependencies at Once

```bash
# Install all missing ROS 2 dependencies
sudo apt update
sudo apt install -y \
  ros-jazzy-behaviortree-cpp-v3 \
  ros-jazzy-robot-localization \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-common \
  ros-jazzy-moveit \
  ros-jazzy-moveit-task-constructor-core \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-vision-msgs \
  ros-jazzy-pcl-ros \
  ros-jazzy-octomap-msgs \
  ros-jazzy-gazebo-ros-pkgs \
  ros-jazzy-rosbag2 \
  ros-jazzy-rviz2
```

## Build Status Summary

- ✅ **21/61 packages** build successfully with base ROS 2 Jazzy installation
- ⏳ **40/61 packages** require additional dependencies
- 🎯 **Core functionality packages** (description, hardware, control) all build successfully

## Recommended Installation Order

For the 2-month development timeline, install dependencies in this order:

### Phase 1 (Weeks 1-3): Core System
```bash
# Already working - no additional dependencies needed!
# These 21 packages provide:
# - Robot description
# - Hardware interfaces
# - Basic control
# - System bringup
```

### Phase 2 (Weeks 4-6): Simulation & State Estimation
```bash
sudo apt install -y \
  ros-jazzy-gazebo-ros-pkgs \
  ros-jazzy-robot-localization \
  ros-jazzy-rviz2
```

### Phase 3 (Weeks 7-8): Advanced Features (Choose One)

**Option A - Manipulation**:
```bash
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-task-constructor-core \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-vision-msgs
```

**Option B - Autonomous Navigation**:
```bash
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-common \
  ros-jazzy-behaviortree-cpp-v3 \
  ros-jazzy-cv-bridge
```

## Current Build Command

To build only packages that currently work:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  humanoid_msgs \
  humanoid_interfaces \
  humanoid_utils \
  humanoid_description \
  humanoid_hardware \
  actuator_interfaces \
  sensor_interfaces \
  safety_monitor \
  diagnostics \
  humanoid_control \
  upper_body_control \
  lower_body_control \
  arm_control \
  hand_control \
  leg_control \
  balance_control \
  locomotion_control \
  bimanual_control \
  slam \
  localization \
  humanoid_bringup
```

## Build All Packages (After Installing Dependencies)

```bash
source /opt/ros/jazzy/setup.bash
colcon build
```
