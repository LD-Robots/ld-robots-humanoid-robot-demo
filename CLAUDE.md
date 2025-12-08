# Humanoid Robot Demo - ROS 2 Jazzy

## Project Overview

This is a comprehensive ROS 2 Jazzy workspace for a humanoid robot system, designed to be built within 2 months. The project includes full-body control, manipulation, locomotion, perception, and simulation capabilities.

## System Architecture

### Timeline
- **Duration**: 2 months development timeline
- **ROS Version**: ROS 2 Jazzy
- **Target Platform**: Humanoid robot with custom actuators (RS04, RS03, RS02 motors)

### Hardware Components
- **Actuators**:
  - RS04: High-torque motors (hip, knee, shoulder joints)
  - RS03: Medium-torque motors (elbow, ankle joints)
  - RS02: Low-torque motors (wrist, hand joints)
- **Sensors**:
  - Cameras (head-mounted)
  - IMU (torso-mounted)
  - Force-torque sensors (feet, wrists)

## Package Structure

Total: **61 ROS 2 packages** organized in **14 categories**

### 1. Common (3 packages)
Foundation packages used across the entire system:
- `humanoid_msgs` - Custom message/service/action definitions
- `humanoid_interfaces` - Base classes and common interfaces
- `humanoid_utils` - Shared utilities (kinematics, transforms, math)

### 2. Robot Description (1 package)
**CONSOLIDATED APPROACH**: Single package with modular xacro files
- `humanoid_description` - Complete URDF/xacro robot model
  - Modular xacro structure: head, torso, arms, hands, legs, sensors
  - Separate visual/collision meshes
  - Configuration files for joint limits and poses
  - Launch files for RViz visualization

### 3. Control (9 packages)
Hierarchical control architecture:
- `humanoid_control` - Whole-body orchestrator
- `upper_body_control` - Arms + torso + head coordination
- `lower_body_control` - Legs + balance coordination
- `arm_control` - Single arm control
- `hand_control` - Hand/gripper control
- `leg_control` - Single leg control
- `balance_control` - CoM, ZMP stabilization
- `locomotion_control` - Gait generation & execution
- `bimanual_control` - Dual-arm coordination

### 4. Planning (7 packages)
Motion planning and trajectory optimization:
- `humanoid_moveit_config` - Whole-body MoveIt configuration
- `upper_body_moveit_config` - Upper body planning
- `lower_body_moveit_config` - Lower body planning
- `manipulation_planning` - Grasp planning
- `locomotion_planning` - Footstep planning
- `humanoid_mtc` - MoveIt Task Constructor integration
- `trajectory_optimization` - TOWR/Crocoddyl integration

### 5. Perception (7 packages)
Vision and environment understanding:
- `vision_pipeline` - Multi-camera fusion orchestrator
- `object_perception` - Detection, tracking, pose estimation
- `scene_understanding` - Segmentation, scene graphs
- `point_cloud_processing` - PCL, clustering, planes
- `semantic_mapping` - OctoMap + semantics
- `visual_servoing` - Vision-based control
- `human_detection` - Human pose & intention recognition

### 6. Localization (3 packages)
State estimation and mapping:
- `state_estimation` - IMU fusion, EKF/UKF
- `slam` - SLAM for unknown environments
- `localization` - Localization in known maps

### 7. Navigation (3 packages)
Humanoid-specific navigation:
- `humanoid_navigation` - Nav2 for bipedal robots
- `footstep_planning` - Discrete footstep planner
- `obstacle_avoidance` - Dynamic obstacle avoidance

### 8. Behavior (5 packages)
High-level autonomy and task execution:
- `behavior_trees` - BehaviorTree.CPP integration
- `task_planning` - High-level task planning
- `manipulation_behaviors` - Pick, place, handover behaviors
- `locomotion_behaviors` - Walk, climb, sit behaviors
- `interaction_behaviors` - Handshake, gesture, collaboration

### 9. Hardware Interface (5 packages)
Low-level hardware drivers and safety:
- `humanoid_hardware` - Main hardware interface
- `actuator_interfaces` - Motor controllers (RS04, RS03, RS02)
- `sensor_interfaces` - IMU, FT sensors, cameras
- `safety_monitor` - Hardware safety limits
- `diagnostics` - Self-test, fault detection

### 10. Simulation (3 packages)
Virtual testing environments:
- `humanoid_gazebo` - Gazebo Harmonic simulation
- `isaac_sim` - NVIDIA Isaac Sim integration (optional)
- `simulation_tools` - Terrain generation, scenarios

### 11. Teleoperation (5 packages)
Various control interfaces:
- `whole_body_teleop` - Full-body control
- `vr_teleop` - VR-based teleoperation
- `keyboard_teleop` - Keyboard control
- `gamepad_teleop` - Gamepad control
- `retargeting` - Motion retargeting

### 12. Bringup (1 package)
System integration and launch:
- `humanoid_bringup` - System launch files (full, minimal, hardware, simulation)

### 13. Applications (4 packages)
Demo tasks and benchmarks:
- `demos` - MTC, locomotion, interaction demos
- `benchmarks` - Performance benchmarks
- `tasks` - Household, warehouse, research tasks
- `manipulation_library` - Reusable manipulation primitives

### 14. Tools (5 packages)
Development and debugging tools:
- `gui_tools` - PyQt5 GUIs (launcher, monitor, BT editor) [Python]
- `diagnostic_tools` - Performance profiler, safety checker
- `calibration_tools` - Kinematic, camera, IMU calibration
- `visualization_tools` - Trajectory, force visualization
- `recording_tools` - Rosbag recording, dataset export

## Development Priority (2-Month Timeline)

### Phase 1: Foundation (Weeks 1-3)
**Critical Path**:
1. `humanoid_msgs`, `humanoid_interfaces`, `humanoid_utils`
2. `humanoid_description` (complete URDF with all components)
3. `humanoid_hardware`, `actuator_interfaces`, `sensor_interfaces`
4. `humanoid_control`, `balance_control`
5. `safety_monitor`
6. `humanoid_bringup`

**Goal**: Robot can stand, basic teleoperation works

### Phase 2: Simulation & Testing (Weeks 4-6)
**Priority**:
1. `humanoid_gazebo` - Full simulation environment
2. `state_estimation` - IMU fusion, odometry
3. `keyboard_teleop`, `gamepad_teleop`
4. `locomotion_control` - Basic walking
5. `diagnostics`

**Goal**: Robot can walk in simulation

### Phase 3: Advanced Features (Weeks 7-8)
**Choose based on requirements**:
- **Option A - Manipulation**: `humanoid_moveit_config`, `manipulation_planning`, `demos`
- **Option B - Autonomy**: `locomotion_planning`, `humanoid_navigation`, `behavior_trees`
- **Option C - Perception**: `vision_pipeline`, `object_perception`, `semantic_mapping`

**Goal**: One advanced capability fully functional

## Key Design Decisions

### Single Description Package
- **Decision**: Use one `humanoid_description` package instead of 9 separate packages
- **Rationale**: Simpler for 2-month timeline, easier maintenance
- **Implementation**: Modular xacro files in subdirectories (head, torso, arms, legs, etc.)

### Joint Naming Convention
```
<side>_<body_part>_<motion>_joint

Examples:
- left_shoulder_pitch_joint
- right_hip_roll_joint
- neck_yaw_joint
```

### Motor Assignment
- **RS04** (high-torque): hip_*, knee_*, shoulder_*
- **RS03** (medium-torque): elbow_*, ankle_*
- **RS02** (low-torque): wrist_*, finger_*

## Building the Workspace

```bash
# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Build all packages
colcon build

# Build specific package
colcon build --packages-select humanoid_description

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Testing

```bash
# Visualize robot in RViz
ros2 launch humanoid_description display.launch.py

# Generate URDF from xacro
xacro src/robot_description/humanoid_description/urdf/humanoid.urdf.xacro > humanoid.urdf

# Validate URDF
check_urdf humanoid.urdf

# Run simulation
ros2 launch humanoid_gazebo simulation.launch.py

# Run hardware interface
ros2 launch humanoid_bringup hardware.launch.py
```

## File Locations

### Robot Description
- **Main URDF**: `src/robot_description/humanoid_description/urdf/humanoid.urdf.xacro`
- **Xacro modules**: `src/robot_description/humanoid_description/xacro/`
- **Meshes**: `src/robot_description/humanoid_description/meshes/`
- **Config**: `src/robot_description/humanoid_description/config/`

### Configuration Files
- **Joint limits**: `src/robot_description/humanoid_description/config/joint_limits.yaml`
- **Initial poses**: `src/robot_description/humanoid_description/config/initial_positions.yaml`

## Dependencies

### System Dependencies
```bash
sudo apt install ros-jazzy-desktop
sudo apt install ros-jazzy-gazebo-ros-pkgs
sudo apt install ros-jazzy-moveit
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-behaviortree-cpp-v3
sudo apt install ros-jazzy-controller-manager
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-joint-state-publisher-gui
```

### Python Dependencies (for gui_tools)
```bash
pip3 install PyQt5
pip3 install matplotlib
pip3 install numpy
```

## Current Status

- ✅ All 61 packages created
- ✅ Package structure defined
- ✅ humanoid_description directory structure ready
- ✅ Common xacro macros created (materials, inertials, transmissions)
- ✅ Main URDF orchestrator template created
- ✅ Configuration files created (joint limits, initial positions)
- ✅ Launch files created (display.launch.py)
- ✅ CMakeLists.txt configured with install directives
- ✅ .gitignore and .gitkeep files added
- ⏳ Individual component xacro files (head, arm, leg, etc.) - **TO BE ADDED BY USER**
- ⏳ Mesh files (.stl, .dae) - **TO BE ADDED**
- ⏳ RViz configuration - **TO BE ADDED**

## Next Steps

1. **Add Robot Geometry**: Create xacro files for each component (head, torso, arms, hands, legs)
2. **Add Meshes**: Import or create 3D meshes for visual and collision geometries
3. **Test URDF**: Build and visualize in RViz
4. **Hardware Interface**: Implement actuator communication protocols
5. **Control**: Implement basic joint position control
6. **Simulation**: Set up Gazebo with physics plugins
7. **Testing**: Validate all subsystems individually, then integrate

## Notes for Claude

- This is a 2-month project with aggressive timeline
- Focus on getting basic functionality working first
- Defer advanced features until core system is stable
- User will create URDF/xacro files themselves
- Package structure is finalized and should not be changed without discussion
- All packages follow ROS 2 conventions and best practices
