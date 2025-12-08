# Project Structure

## Overview

This project uses a **subsystem-based architecture** where packages are organized by functionality into separate directories.

## Directory Tree

```
src/
├── bringup/                    # System Integration
│   └── humanoid_bringup/       - Launch files and startup
├── control/                    # Control Systems
│   └── humanoid_controllers/   - ros2_control configurations
├── description/                # Robot Models
│   └── humanoid_description/   - URDF/Xacro definitions
├── hardware/                   # Hardware Interfaces
│   └── humanoid_hardware/      - Hardware abstraction layer
├── locomotion/                 # Walking & Balance
│   └── humanoid_locomotion/    - Gait generation & balance
├── manipulation/               # Arm Control
│   └── humanoid_manipulation/  - Arm planning & grippers
├── perception/                 # Sensing
│   └── humanoid_perception/    - Camera & IMU processing
└── simulation/                 # Virtual Testing
    └── humanoid_simulation/    - Gazebo integration
```

## Quick Navigation

- **Want to modify robot appearance?** → `src/description/humanoid_description/urdf/`
- **Want to tune controllers?** → `src/control/humanoid_controllers/config/`
- **Want to add hardware driver?** → `src/hardware/humanoid_hardware/src/`
- **Want to modify walking?** → `src/locomotion/humanoid_locomotion/scripts/`
- **Want to change arm behavior?** → `src/manipulation/humanoid_manipulation/scripts/`
- **Want to process camera/IMU?** → `src/perception/humanoid_perception/scripts/`
- **Want to modify simulation?** → `src/simulation/humanoid_simulation/`
- **Want to change startup?** → `src/bringup/humanoid_bringup/launch/`

## Adding New Packages

See each subsystem's README for guidance on adding packages:
- `src/description/README.md`
- `src/control/README.md`
- `src/hardware/README.md`
- etc.

## Benefits

✅ Organized by functionality
✅ Easy to navigate
✅ Scales to 50+ packages
✅ Professional structure
✅ Clear where new code belongs

See [ARCHITECTURE.md](ARCHITECTURE.md) for detailed architecture information.
