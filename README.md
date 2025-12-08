# Humanoid Robot - ROS2 Jazzy

A modern, modular humanoid robot implementation using ROS2 Jazzy with professional subsystem-based architecture.

## Architecture Overview

This project uses a **subsystem-based architecture** where packages are organized by functionality:

```
src/
├── bringup/        System integration
├── control/        Low-level control
├── description/    Robot models
├── hardware/       Hardware drivers
├── locomotion/     Walking & balance
├── manipulation/   Arm control
├── perception/     Sensors & vision
└── simulation/     Virtual testing
```

Each subsystem contains one or more related packages and can grow independently. See [STRUCTURE.md](STRUCTURE.md) for details.

## Features

- **29 DOF humanoid robot** (head, torso, 2 arms, 2 legs, grippers)
- **Modern ros2_control** integration
- **Multiple operation modes** (real hardware, fake hardware, Gazebo)
- **Complete sensor suite** (IMU, camera, foot contacts)
- **Walking and balance** with gait generation
- **Manipulation capabilities** with arm planning
- **Perception pipeline** with camera and IMU processing

## Quick Start

### Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Visualize

```bash
ros2 launch description/humanoid_description view_robot.launch.py
```

### Simulate

```bash
ros2 launch simulation/humanoid_simulation simulation.launch.py
```

### Test with Fake Hardware

```bash
ros2 launch bringup/humanoid_bringup fake_robot.launch.py
```

## Documentation

- **[QUICKSTART.md](QUICKSTART.md)** - Get started in 5 minutes
- **[STRUCTURE.md](STRUCTURE.md)** - Project organization
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - Detailed architecture
- **[SETUP.md](SETUP.md)** - Installation guide
- **[USAGE.md](USAGE.md)** - Usage examples
- **[FEATURES.md](FEATURES.md)** - Complete feature list

## Subsystems

Each subsystem has its own README with future growth ideas:

- [bringup/README.md](src/bringup/README.md) - System integration
- [control/README.md](src/control/README.md) - Controllers
- [description/README.md](src/description/README.md) - Robot models
- [hardware/README.md](src/hardware/README.md) - Hardware drivers
- [locomotion/README.md](src/locomotion/README.md) - Walking & balance
- [manipulation/README.md](src/manipulation/README.md) - Arm control
- [perception/README.md](src/perception/README.md) - Sensors & vision
- [simulation/README.md](src/simulation/README.md) - Virtual testing

## Why This Architecture?

✅ **Scalable** - Add packages easily within each subsystem
✅ **Organized** - Related functionality grouped together
✅ **Professional** - Follows major robotics projects (PR2, TIAGo)
✅ **Clear** - Obvious where new code belongs
✅ **Maintainable** - Easy to navigate and update

## Packages

- `humanoid_description` - Full URDF/Xacro robot model
- `humanoid_hardware` - ros2_control hardware interface
- `humanoid_controllers` - Controller configurations
- `humanoid_bringup` - Launch files and integration
- `humanoid_simulation` - Gazebo simulation
- `humanoid_perception` - Camera and IMU processing
- `humanoid_locomotion` - Gait generation and balance
- `humanoid_manipulation` - Arm control and grippers

## Requirements

- ROS2 Jazzy
- Python 3.10+
- Gazebo Harmonic (for simulation)

## License

MIT License

## Contributing

This structure makes it easy to contribute:

1. Choose the right subsystem for your feature
2. Create a new package or modify existing one
3. Follow the patterns in that subsystem
4. Update the subsystem's README

See individual subsystem READMEs for specific guidance.
