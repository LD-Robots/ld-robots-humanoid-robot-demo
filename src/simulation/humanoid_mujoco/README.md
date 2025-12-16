# Humanoid MuJoCo Simulation

MuJoCo-based physics simulation for humanoid robot with focus on balance control and bipedal locomotion.

## Overview

This package provides a high-fidelity physics simulation using MuJoCo (Multi-Joint dynamics with Contact) for humanoid robot development. MuJoCo is chosen for its:

- **Fast and accurate contact dynamics** - Essential for bipedal walking
- **Efficient computation** - Enables real-time control at high frequencies (>100 Hz)
- **Superior stability** - Robust handling of constrained dynamics
- **Built-in sensors** - IMU, force-torque, and kinematic sensors

## Features

- Full humanoid robot model in MuJoCo MJCF format
- ROS 2 integration with joint state, IMU, and force sensor publishing
- Interactive MuJoCo viewer for visualization
- Balance control integration
- Walking controller support
- Configurable physics parameters
- RViz visualization support

## Installation

### Install MuJoCo

```bash
pip3 install mujoco
```

### Build the Package

```bash
cd ~/your_workspace
colcon build --packages-select humanoid_mujoco
source install/setup.bash
```

## Quick Start

### 1. Generate MuJoCo Model from URDF

```bash
# Generate template MJCF model
ros2 run humanoid_mujoco urdf_to_mjcf.py --template --output models/humanoid.xml

# Or convert from URDF (requires URDF file)
ros2 run humanoid_mujoco urdf_to_mjcf.py \
  --urdf /path/to/humanoid.urdf \
  --output models/humanoid.xml
```

### 2. Test Model with Visualizer

```bash
# Launch standalone visualizer
ros2 run humanoid_mujoco mujoco_visualizer.py \
  $(ros2 pkg prefix humanoid_mujoco)/share/humanoid_mujoco/models/humanoid.xml

# With balance perturbations
ros2 run humanoid_mujoco mujoco_visualizer.py \
  $(ros2 pkg prefix humanoid_mujoco)/share/humanoid_mujoco/models/humanoid.xml \
  --test-balance

# Print model info only
ros2 run humanoid_mujoco mujoco_visualizer.py \
  $(ros2 pkg prefix humanoid_mujoco)/share/humanoid_mujoco/models/humanoid.xml \
  --info
```

### 3. Launch Full Simulation with ROS 2

```bash
# Basic simulation with MuJoCo viewer and RViz
ros2 launch humanoid_mujoco mujoco_simulation.launch.py

# Headless simulation (no MuJoCo viewer)
ros2 launch humanoid_mujoco mujoco_simulation.launch.py use_viewer:=false

# Balance control demo
ros2 launch humanoid_mujoco balance_demo.launch.py

# Walking demo
ros2 launch humanoid_mujoco walking_demo.launch.py
```

## Package Structure

```
humanoid_mujoco/
├── config/                      # Configuration files
│   ├── simulation_params.yaml   # MuJoCo simulation settings
│   ├── balance_controller.yaml  # Balance control parameters
│   ├── walking_controller.yaml  # Walking control parameters
│   └── mujoco_visualization.rviz # RViz configuration
├── launch/                      # Launch files
│   ├── mujoco_simulation.launch.py
│   ├── balance_demo.launch.py
│   └── walking_demo.launch.py
├── models/                      # MuJoCo MJCF models
│   └── humanoid.xml            # Main robot model
├── worlds/                      # Simulation environments
├── scripts/                     # Python executables
│   ├── mujoco_simulator.py     # Main ROS 2 simulator node
│   ├── mujoco_visualizer.py    # Standalone visualizer
│   └── urdf_to_mjcf.py         # URDF to MJCF converter
├── src/                         # C++ source files
│   └── mujoco_ros_bridge.cpp   # Hardware interface bridge
└── include/                     # C++ headers
```

## ROS 2 Topics

### Published Topics

- `/joint_states` (sensor_msgs/JointState) - Joint positions, velocities, efforts
- `/imu/data` (sensor_msgs/Imu) - IMU sensor data
- `/left_foot/wrench` (geometry_msgs/WrenchStamped) - Left foot force/torque
- `/right_foot/wrench` (geometry_msgs/WrenchStamped) - Right foot force/torque
- `/torso/pose` (geometry_msgs/PoseStamped) - Torso pose
- `/odom` (nav_msgs/Odometry) - Robot odometry
- `/tf` - Transform tree

### Subscribed Topics

- `/joint_commands` (std_msgs/Float64MultiArray) - Joint position commands

## Configuration Parameters

### Simulation Parameters

```yaml
mujoco_simulator:
  ros__parameters:
    timestep: 0.002          # Physics timestep (seconds)
    publish_rate: 100.0      # State publishing rate (Hz)
    realtime_factor: 1.0     # Simulation speed
    solver_iterations: 50    # Newton solver iterations
```

### Balance Control

```yaml
balance_controller:
  ros__parameters:
    com_height_desired: 0.85   # Desired CoM height (m)
    zmp_control_enabled: true  # Enable ZMP control
    ankle_strategy_enabled: true
    hip_strategy_enabled: true
    stepping_strategy_enabled: true
```

### Walking Control

```yaml
locomotion_controller:
  ros__parameters:
    gait_type: "walk"
    step_frequency: 0.5        # Steps per second
    step_length: 0.15          # Step length (m)
    step_width: 0.2            # Step width (m)
    max_forward_velocity: 0.5  # Max speed (m/s)
```

## MuJoCo Model Development

### MJCF File Structure

The humanoid model is defined in MuJoCo's MJCF XML format:

```xml
<mujoco model="humanoid">
  <compiler angle="radian" meshdir="meshes"/>

  <option timestep="0.002" solver="Newton" gravity="0 0 -9.81"/>

  <worldbody>
    <body name="torso" pos="0 0 1.0">
      <freejoint name="root"/>
      <geom type="box" size="0.15 0.1 0.25"/>
      <!-- Additional body parts -->
    </body>
  </worldbody>

  <actuator>
    <position joint="hip_pitch" kp="500"/>
    <!-- Additional actuators -->
  </actuator>

  <sensor>
    <accelerometer site="imu_site"/>
    <force site="foot_site"/>
    <!-- Additional sensors -->
  </sensor>
</mujoco>
```

### Key MJCF Features for Humanoids

1. **Contact Dynamics**: Precise ground contact modeling
2. **Actuator Models**: Position, velocity, or torque control
3. **Sensor Suite**: IMU, force-torque, joint sensors
4. **Keyframes**: Predefined poses (standing, sitting, etc.)
5. **Visual Assets**: Meshes, textures, materials

## Comparison: MuJoCo vs Gazebo

| Feature | MuJoCo | Gazebo |
|---------|--------|--------|
| Contact Solver | Convex optimization | LCP/iterative |
| Speed | Very fast (~10x) | Moderate |
| Stability | Excellent | Good |
| ROS Integration | Custom bridge | Native (gazebo_ros) |
| Visualization | Built-in viewer | Integrated 3D view |
| Best For | Legged robots, manipulation | General robotics |

## Troubleshooting

### MuJoCo Not Found

```bash
pip3 install mujoco
# Verify installation
python3 -c "import mujoco; print(mujoco.__version__)"
```

### Model Fails to Load

Check model with visualizer:
```bash
ros2 run humanoid_mujoco mujoco_visualizer.py models/humanoid.xml --info
```

### Poor Contact Simulation

Adjust contact parameters in MJCF:
```xml
<option>
  <flag warmstart="enable"/>
</option>
<default>
  <geom friction="1.0 0.005 0.0001" solimp="0.9 0.95 0.001"/>
</default>
```

## Development Workflow

1. **Model Creation**: Design robot in MJCF or convert from URDF
2. **Standalone Testing**: Use `mujoco_visualizer.py` for quick iteration
3. **ROS Integration**: Launch with `mujoco_simulation.launch.py`
4. **Controller Development**: Build controllers using published sensor data
5. **Performance Tuning**: Adjust physics parameters in config files

## References

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MuJoCo Python Bindings](https://github.com/deepmind/mujoco)
- [MJCF XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [MuJoCo Menagerie](https://github.com/deepmind/mujoco_menagerie) - Example robot models

## License

Apache 2.0

## Contributing

For issues and feature requests, please use the issue tracker.
