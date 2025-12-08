# Humanoid Description Package

This package contains the complete URDF/Xacro robot description for the humanoid robot.

## Directory Structure

```
humanoid_description/
├── urdf/                           # Main URDF files
│   └── humanoid.urdf.xacro        # Main robot orchestrator (includes all xacro files)
│
├── xacro/                          # Modular xacro component files
│   ├── common/                     # Common macros and properties
│   │   ├── materials.xacro        # Color/material definitions
│   │   ├── inertials.xacro        # Inertial property macros
│   │   └── transmission.xacro     # ros2_control transmission macros
│   ├── head/                       # Head components
│   │   └── head.xacro             # Head assembly with neck joints
│   ├── torso/                      # Torso/spine components
│   │   └── torso.xacro            # Torso assembly with spine joints
│   ├── arms/                       # Arm components (reusable)
│   │   ├── arm.xacro              # Single arm macro (left/right)
│   │   └── shoulder.xacro         # Shoulder joint assembly
│   ├── hands/                      # Hand/gripper components (reusable)
│   │   └── hand.xacro             # Dexterous hand macro
│   ├── legs/                       # Leg components (reusable)
│   │   ├── leg.xacro              # Single leg macro (left/right)
│   │   └── hip.xacro              # Hip joint assembly
│   └── sensors/                    # Sensor components
│       ├── cameras.xacro          # Camera sensors
│       ├── imu.xacro              # IMU sensor
│       └── force_torque.xacro     # Force-torque sensors
│
├── meshes/                         # 3D mesh files
│   ├── visual/                     # High-poly meshes for visualization
│   │   ├── head/
│   │   ├── torso/
│   │   ├── arms/
│   │   ├── hands/
│   │   └── legs/
│   └── collision/                  # Low-poly meshes for collision detection
│       ├── head/
│       ├── torso/
│       ├── arms/
│       ├── hands/
│       └── legs/
│
├── config/                         # Configuration files
│   ├── joint_limits.yaml          # Joint position/velocity/effort limits
│   ├── physical_properties.yaml   # Link masses, inertias, CoM
│   ├── ros2_control.yaml          # Hardware interface configuration
│   └── initial_positions.yaml     # Default joint positions
│
├── launch/                         # Launch files
│   ├── display.launch.py          # RViz visualization
│   ├── load_urdf.launch.py        # Load URDF to parameter server
│   └── gazebo.launch.py           # Gazebo simulation launch
│
└── rviz/                           # RViz configuration files
    └── humanoid.rviz              # Default RViz config
```

## Usage

### Load and View in RViz
```bash
ros2 launch humanoid_description display.launch.py
```

### Generate URDF from Xacro
```bash
xacro src/robot_description/humanoid_description/urdf/humanoid.urdf.xacro > humanoid.urdf
```

### Validate URDF
```bash
check_urdf humanoid.urdf
```

### View URDF Tree
```bash
urdf_to_graphviz humanoid.urdf
```

## Xacro Modularity

The robot description uses a hierarchical approach:

1. **humanoid.urdf.xacro** - Main orchestrator
   - Includes common properties and macros
   - Instantiates torso (base)
   - Instantiates head
   - Instantiates left/right arms with hands
   - Instantiates left/right legs
   - Includes sensors

2. **Reusable macros** - arm.xacro, leg.xacro, hand.xacro
   - Parameterized by side (left/right)
   - Consistent naming convention: `${prefix}_component_link`

## Actuator Naming Convention

### Motors
- RS04: High-torque (hip, knee, shoulder)
- RS03: Medium-torque (elbow, ankle)
- RS02: Low-torque (wrist, fingers)

### Joint Naming Pattern
```
<side>_<body_part>_<motion>_joint

Examples:
- left_shoulder_pitch_joint
- right_hip_roll_joint
- neck_yaw_joint
```

## Next Steps

1. Create main humanoid.urdf.xacro orchestrator
2. Define common materials and properties
3. Create modular xacro files for each component
4. Add mesh files (STL/DAE)
5. Configure joint limits and physical properties
6. Create launch files for visualization and simulation
7. Test in RViz and Gazebo

## Dependencies

- urdf
- xacro
- robot_state_publisher
- joint_state_publisher
- rviz2
- gazebo_ros (for simulation)
