# Quick Start Guide

## 5-Minute Setup

### 1. Prerequisites
Ensure you have ROS2 Jazzy installed. If not, see [SETUP.md](SETUP.md).

```bash
source /opt/ros/jazzy/setup.bash
```

### 2. Build the Workspace

```bash
cd ~/Documents/GitHub/ld-robots-humanoid-robot-demo
colcon build --symlink-install
source install/setup.bash
```

### 3. Visualize in RViz

```bash
ros2 launch humanoid_description view_robot.launch.py
```

Use the GUI sliders to move joints and see the robot in RViz.

## Test with Fake Hardware

```bash
# Terminal 1
ros2 launch humanoid_bringup fake_robot.launch.py

# Terminal 2 (optional - view in RViz)
rviz2
```

## Run Full Simulation

```bash
ros2 launch humanoid_simulation simulation.launch.py
```

This starts:
- Gazebo with the humanoid robot
- All controllers
- Robot state publisher

## Send Commands

### Control the Head
```bash
ros2 topic pub /head_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['neck_pitch', 'neck_yaw'], points: [{positions: [0.5, 0.3], time_from_start: {sec: 1}}]}"
```

### Control a Gripper
```bash
# Open gripper
ros2 topic pub /left_gripper/command std_msgs/msg/Float64 "{data: 1.0}" --once

# Close gripper
ros2 topic pub /left_gripper/command std_msgs/msg/Float64 "{data: 0.0}" --once
```

### Walk Forward (in simulation)
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1}, angular: {z: 0.0}}"
```

## Next Steps

- Read [USAGE.md](USAGE.md) for detailed examples
- Check [ARCHITECTURE.md](ARCHITECTURE.md) to understand the system
- See [FEATURES.md](FEATURES.md) for complete feature list
- Customize robot in `src/humanoid_description/urdf/`
- Tune controllers in `src/humanoid_controllers/config/`

## Troubleshooting

**Build fails**: Run `rosdep install --from-paths src --ignore-src -r -y`

**Controllers don't start**: Check that ros2_control packages are installed

**Gazebo crashes**: Ensure Gazebo Harmonic (not Classic) is installed

**Python scripts fail**: Run `chmod +x src/*/scripts/*.py`

## Documentation

- [README.md](README.md) - Overview
- [SETUP.md](SETUP.md) - Detailed setup
- [USAGE.md](USAGE.md) - Usage examples
- [ARCHITECTURE.md](ARCHITECTURE.md) - System architecture
- [FEATURES.md](FEATURES.md) - Feature list
