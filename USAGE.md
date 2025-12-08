# Usage Guide

## Visualization in RViz

View the robot model without controllers:

```bash
ros2 launch humanoid_description view_robot.launch.py
```

This will open RViz with the robot model and a GUI to control joint positions.

## Testing with Fake Hardware

Test the system without a real robot or simulation:

```bash
# Terminal 1: Launch fake hardware
ros2 launch humanoid_bringup fake_robot.launch.py

# Terminal 2: View in RViz
ros2 launch humanoid_description view_robot.launch.py use_fake_hardware:=true
```

## Gazebo Simulation

Launch the full simulation:

```bash
ros2 launch humanoid_simulation simulation.launch.py
```

Options:
- `gui:=false` - Run headless (no Gazebo GUI)
- `world:=custom.sdf` - Use custom world file

## Running Individual Systems

### Perception System

```bash
ros2 launch humanoid_perception perception.launch.py
```

Topics:
- `/camera/image_raw` - Raw camera images
- `/camera/image_processed` - Processed images
- `/imu/data` - Raw IMU data
- `/imu/data_filtered` - Filtered IMU data

### Locomotion System

```bash
ros2 launch humanoid_locomotion locomotion.launch.py
```

Control walking:
```bash
# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Manipulation System

```bash
ros2 launch humanoid_manipulation manipulation.launch.py
```

Control grippers:
```bash
# Open left gripper (1.0 = fully open)
ros2 topic pub /left_gripper/command std_msgs/msg/Float64 "{data: 1.0}"

# Close left gripper (0.0 = fully closed)
ros2 topic pub /left_gripper/command std_msgs/msg/Float64 "{data: 0.0}"
```

## Controller Management

List active controllers:
```bash
ros2 control list_controllers
```

Load a controller:
```bash
ros2 control load_controller <controller_name>
```

Start/stop controllers:
```bash
ros2 control set_controller_state <controller_name> active
ros2 control set_controller_state <controller_name> inactive
```

## Monitoring and Debugging

### View TF tree
```bash
ros2 run tf2_tools view_frames
```

### Monitor joint states
```bash
ros2 topic echo /joint_states
```

### Check controller status
```bash
ros2 control list_controllers
```

### View IMU data
```bash
ros2 topic echo /imu/data
```

### Monitor camera feed
```bash
ros2 run rqt_image_view rqt_image_view
```

## Common Issues

### Controllers fail to start
- Ensure hardware interface is properly configured
- Check that all joint names match between URDF and controller config
- Verify ros2_control is properly loaded

### Simulation doesn't start
- Ensure Gazebo Harmonic is installed
- Check that world file exists
- Verify ros_gz_sim is installed

### Python scripts not executable
```bash
chmod +x src/*/scripts/*.py
```

### Missing dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Advanced Usage

### Custom Hardware Configuration

Edit hardware parameters in:
- `src/humanoid_description/urdf/ros2_control.urdf.xacro`
- `src/humanoid_hardware/src/humanoid_hardware_interface.cpp`

### Custom Controller Parameters

Edit controller configurations in:
- `src/humanoid_controllers/config/controllers.yaml`

### Custom Gaits

Modify gait parameters in:
- `src/humanoid_locomotion/scripts/gait_generator.py`

Parameters:
- `step_length` - Length of each step (meters)
- `step_height` - Height to lift foot (meters)
- `step_duration` - Time for each step (seconds)
- `double_support_ratio` - Ratio of time both feet are on ground

## Integration with MoveIt2

To add motion planning capabilities:

```bash
# Install MoveIt2
sudo apt install ros-jazzy-moveit

# Create MoveIt2 configuration (coming soon)
ros2 launch humanoid_manipulation moveit.launch.py
```

## Hardware Integration

To connect real hardware:

1. Implement hardware communication in `humanoid_hardware/src/humanoid_hardware_interface.cpp`
2. Configure serial port/connection parameters
3. Launch with real hardware:

```bash
ros2 launch humanoid_bringup robot.launch.py
```
