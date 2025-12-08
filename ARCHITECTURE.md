# Architecture Overview

## System Design

This humanoid robot implementation follows a **modular, subsystem-based architecture** with clear separation of concerns. Each major subsystem has its own directory containing related packages, making the codebase scalable and maintainable.

## Improved Package Structure

```
src/
├── description/               # Robot model definitions
│   └── humanoid_description/
├── control/                   # Low-level control
│   └── humanoid_controllers/
├── hardware/                  # Hardware interfaces
│   └── humanoid_hardware/
├── perception/                # Sensing and vision
│   └── humanoid_perception/
├── locomotion/                # Walking and balance
│   └── humanoid_locomotion/
├── manipulation/              # Arm control and grasping
│   └── humanoid_manipulation/
├── simulation/                # Simulation environments
│   └── humanoid_simulation/
└── bringup/                   # System integration
    └── humanoid_bringup/
```

### Why This Structure?

1. **Scalability** - Easy to add new packages within each subsystem
2. **Organization** - Related functionality grouped together
3. **Clarity** - Clear boundaries between subsystems
4. **Professional** - Follows patterns used in major robotics projects (PR2, TIAGo, etc.)
5. **Multi-robot Ready** - Easy to add packages for robot variants

## Subsystem Descriptions

### 1. Description Subsystem (`description/`)

**Purpose**: Robot model definitions and visualization

**Current Packages**:
- `humanoid_description` - URDF/Xacro model, sensors, RViz config

**Future Growth**:
- `humanoid_meshes` - 3D visual/collision meshes
- `humanoid_materials` - Material definitions
- `humanoid_variants` - Robot configuration variants
- `humanoid_tools` - URDF validation tools

### 2. Control Subsystem (`control/`)

**Purpose**: Joint-level control and controller management

**Current Packages**:
- `humanoid_controllers` - ros2_control configurations

**Future Growth**:
- `humanoid_control_msgs` - Custom control messages
- `humanoid_admittance_control` - Compliant control
- `humanoid_whole_body_control` - Coordinated control
- `humanoid_balance_control` - Balance controllers

### 3. Hardware Subsystem (`hardware/`)

**Purpose**: Hardware abstraction and device drivers

**Current Packages**:
- `humanoid_hardware` - ros2_control hardware interface

**Future Growth**:
- `humanoid_motor_drivers` - Motor-specific drivers
- `humanoid_sensor_drivers` - Sensor drivers
- `humanoid_can_interface` - CAN bus support
- `humanoid_diagnostics` - Health monitoring

### 4. Perception Subsystem (`perception/`)

**Purpose**: Sensor processing and state estimation

**Current Packages**:
- `humanoid_perception` - Camera and IMU processing

**Future Growth**:
- `humanoid_vision` - Computer vision (detection, tracking)
- `humanoid_sensor_fusion` - Multi-sensor fusion
- `humanoid_localization` - SLAM and localization
- `humanoid_object_recognition` - Object detection

### 5. Locomotion Subsystem (`locomotion/`)

**Purpose**: Walking, balance, and navigation

**Current Packages**:
- `humanoid_locomotion` - Gait generation and balance

**Future Growth**:
- `humanoid_footstep_planner` - Footstep planning
- `humanoid_zmp_control` - ZMP-based control
- `humanoid_navigation` - Nav2 integration
- `humanoid_terrain_adaptation` - Terrain handling

### 6. Manipulation Subsystem (`manipulation/`)

**Purpose**: Arm control and object manipulation

**Current Packages**:
- `humanoid_manipulation` - Arm planning and gripper control

**Future Growth**:
- `humanoid_moveit_config` - MoveIt2 setup
- `humanoid_grasping` - Grasp planning
- `humanoid_bimanual` - Dual-arm coordination
- `humanoid_object_manipulation` - High-level skills

### 7. Simulation Subsystem (`simulation/`)

**Purpose**: Virtual environments and testing

**Current Packages**:
- `humanoid_simulation` - Gazebo setup

**Future Growth**:
- `humanoid_gazebo_plugins` - Custom plugins
- `humanoid_worlds` - World files and scenarios
- `humanoid_test_scenarios` - Automated tests
- `humanoid_isaac_sim` - Isaac Sim support (optional)

### 8. Bringup Subsystem (`bringup/`)

**Purpose**: System startup and integration

**Current Packages**:
- `humanoid_bringup` - Launch files and integration

**Future Growth**:
- `humanoid_config` - Configuration management
- `humanoid_monitoring` - System monitoring
- `humanoid_safety` - Safety systems
- `humanoid_calibration` - Calibration tools

## Layer Architecture

### Layer 1: Hardware Interface
- **humanoid_hardware**: Abstracts hardware communication
- Implements ros2_control SystemInterface
- Supports multiple backends (real HW, fake HW, simulation)

### Layer 2: Control
- **humanoid_controllers**: Joint-level control
- Uses ros2_control framework
- Position, velocity, and effort control modes
- Modular controller configuration

### Layer 3: Motion Generation
- **humanoid_locomotion**: Gait generation and balance
- **humanoid_manipulation**: Arm trajectory planning
- High-level motion primitives

### Layer 4: Perception
- **humanoid_perception**: Sensor data processing
- Camera image processing
- IMU filtering and fusion

### Layer 5: Application
- Task planning and execution
- Behavior coordination
- User interfaces

## Robot Model Architecture

### Kinematic Structure
```
base_link
└── torso_link
    ├── head
    │   ├── neck_pitch
    │   └── neck_yaw
    ├── left_arm
    │   ├── shoulder (pitch, roll, yaw)
    │   ├── elbow (pitch)
    │   ├── wrist (pitch, roll)
    │   └── gripper
    ├── right_arm (symmetric)
    ├── left_leg
    │   ├── hip (yaw, roll, pitch)
    │   ├── knee (pitch)
    │   └── ankle (pitch, roll)
    └── right_leg (symmetric)
```

### Degrees of Freedom
- Head: 2 DOF
- Torso: 1 DOF
- Arms: 7 DOF each (6 + gripper)
- Legs: 6 DOF each
- **Total: 29 DOF**

## Control Architecture

### ros2_control Integration
```
Hardware Interface
    ↓
Controller Manager
    ↓
Controllers
    ├── joint_state_broadcaster
    ├── imu_sensor_broadcaster
    ├── head_controller
    ├── torso_controller
    ├── left/right_arm_controller
    ├── left/right_leg_controller
    └── left/right_gripper_controller
```

### Controller Types
- **JointTrajectoryController**: Position-based trajectory following
- **JointStateBroadcaster**: Publishes joint states
- **IMUSensorBroadcaster**: Publishes IMU data

## Communication Patterns

### Topics
- `/joint_states` - Current joint positions/velocities
- `/imu/data` - Raw IMU measurements
- `/camera/image_raw` - Camera images
- `/cmd_vel` - Velocity commands for walking
- Controller-specific command topics

### Actions
- `FollowJointTrajectory` - Execute arm/leg trajectories
- Used for high-level motion commands

### Services
- Controller management (load, unload, configure)
- Hardware interface control

## Data Flow

### Perception Pipeline
```
Sensors (Camera, IMU)
    ↓
Hardware Interface
    ↓
Sensor Processing (filtering, feature extraction)
    ↓
Higher-level perception (object detection, localization)
```

### Control Pipeline
```
High-level Commands (e.g., "walk forward")
    ↓
Motion Planning (gait generation, trajectory planning)
    ↓
Controller Commands (joint trajectories)
    ↓
ros2_control Controllers
    ↓
Hardware Interface
    ↓
Actuators
```

### Balance Control Loop
```
IMU Sensor
    ↓
Balance Controller (PD control)
    ↓
Ankle/Hip Corrections
    ↓
Joint Controllers
```

## Design Principles

### Modularity
- Each package has a single, well-defined responsibility
- Packages can be developed and tested independently
- Easy to add/remove functionality

### Abstraction
- Hardware details hidden behind ros2_control interface
- Same code works with real hardware, fake hardware, or simulation
- Controllers are hardware-agnostic

### Scalability
- Easy to add new sensors or actuators
- Modular controllers can be enabled/disabled
- Configuration-driven behavior

### Maintainability
- Clear separation of concerns
- Standard ROS2 patterns and tools
- Comprehensive documentation

## Extension Points

### Adding New Sensors
1. Add sensor to URDF in `humanoid_description`
2. Add Gazebo plugin if using simulation
3. Create processing node in `humanoid_perception`

### Adding New Controllers
1. Define controller in `controllers.yaml`
2. Add to launch files
3. Implement custom controller if needed

### Custom Gaits
1. Modify `gait_generator.py` in `humanoid_locomotion`
2. Adjust parameters for step length, height, timing
3. Implement new trajectory generation algorithms

### Hardware Integration
1. Implement communication in `humanoid_hardware_interface.cpp`
2. Handle sensor reading and command writing
3. Configure parameters (serial port, baud rate, etc.)

## Technology Stack

- **ROS2 Jazzy**: Core framework
- **ros2_control**: Control framework
- **Gazebo Harmonic**: Simulation
- **Python 3**: High-level logic
- **C++**: Performance-critical components
- **Xacro**: Modular URDF description
- **OpenCV**: Vision processing (optional integration)

## Adding New Packages

### Within Existing Subsystem

Example: Adding object detection to perception

```bash
# Create new package
cd src/perception
ros2 pkg create humanoid_object_detection --dependencies rclpy sensor_msgs

# Package automatically grouped with perception packages
```

### New Subsystem

Example: Adding task planning subsystem

```bash
# Create new subsystem directory
mkdir -p src/planning

# Add packages
cd src/planning
ros2 pkg create humanoid_task_planning --dependencies rclpy

# Add README
cat > README.md << EOF
# Planning Subsystem
Task and behavior planning
EOF
```

## Multi-Robot Support

This structure makes it easy to support multiple robots:

```
src/
├── description/
│   ├── humanoid_description/      # Current robot
│   ├── humanoid_mini_description/ # Smaller variant
│   └── humanoid_pro_description/  # Advanced variant
└── ...
```

## Comparison with Common Patterns

### Our Pattern (Subsystem-based)
```
src/
├── control/
│   └── robot_controllers/
├── perception/
│   └── robot_perception/
```

**Benefits**:
- Scales to large projects
- Clear organization
- Easy to navigate
- Professional structure

### Flat Pattern (Not recommended)
```
src/
├── robot_controllers/
├── robot_perception/
```

**Issues**:
- Hard to navigate with many packages
- No logical grouping
- Scales poorly

## Future Enhancements

- MoveIt2 integration for advanced motion planning
- Navigation2 integration for autonomous navigation
- Sensor fusion (Kalman filtering, SLAM)
- Machine learning for adaptive control
- Behavior trees for task planning
- Multi-robot coordination

## Benefits of This Architecture

✅ **Professional** - Used by major robotics projects
✅ **Scalable** - Can grow to 50+ packages easily
✅ **Organized** - Everything has its place
✅ **Clear** - Obvious where new code goes
✅ **Maintainable** - Easy to find and update code
✅ **Flexible** - Supports multiple robots/variants
✅ **Documentation** - Each subsystem documented

---

**This architecture is production-ready and follows industry best practices for large-scale robotics systems.**
