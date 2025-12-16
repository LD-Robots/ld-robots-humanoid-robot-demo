# MuJoCo Simulation Quick Start Guide

## Prerequisites

### 1. Install MuJoCo Python Bindings

```bash
pip3 install mujoco numpy
```

Verify installation:
```bash
python3 -c "import mujoco; print(f'MuJoCo {mujoco.__version__} installed successfully')"
```

### 2. Build the Package

```bash
cd ~/ld-robots-humanoid-robot-demo
colcon build --packages-select humanoid_mujoco
source install/setup.bash
```

## Getting Started

### Test 1: Visualize the Model

Test the MuJoCo model with the standalone visualizer:

```bash
# Get the model path
MODEL_PATH=$(ros2 pkg prefix humanoid_mujoco)/share/humanoid_mujoco/models/humanoid.xml

# Launch visualizer
ros2 run humanoid_mujoco mujoco_visualizer.py $MODEL_PATH
```

**Viewer Controls:**
- **Left mouse**: Rotate view
- **Right mouse**: Pan view
- **Scroll**: Zoom
- **Space**: Pause/Resume
- **Esc**: Exit

### Test 2: Print Model Information

```bash
MODEL_PATH=$(ros2 pkg prefix humanoid_mujoco)/share/humanoid_mujoco/models/humanoid.xml
ros2 run humanoid_mujoco mujoco_visualizer.py $MODEL_PATH --info
```

This will show:
- Number of bodies, joints, actuators
- Joint names and types
- Sensor configuration
- Physics parameters

### Test 3: Balance Testing with Perturbations

Test the model's passive stability with random perturbations:

```bash
MODEL_PATH=$(ros2 pkg prefix humanoid_mujoco)/share/humanoid_mujoco/models/humanoid.xml
ros2 run humanoid_mujoco mujoco_visualizer.py $MODEL_PATH --test-balance
```

The robot will receive random horizontal forces every 5 seconds to test balance.

### Test 4: Full ROS 2 Simulation

Launch the complete simulation with ROS 2 integration:

```bash
# With MuJoCo viewer and RViz
ros2 launch humanoid_mujoco mujoco_simulation.launch.py

# Headless (no MuJoCo viewer, RViz only)
ros2 launch humanoid_mujoco mujoco_simulation.launch.py use_viewer:=false

# No visualization at all (for benchmarking)
ros2 launch humanoid_mujoco mujoco_simulation.launch.py use_viewer:=false use_rviz:=false
```

### Test 5: Check ROS Topics

In a new terminal (while simulation is running):

```bash
# List all topics
ros2 topic list

# View joint states
ros2 topic echo /joint_states

# View IMU data
ros2 topic echo /imu/data

# View foot forces
ros2 topic echo /left_foot/wrench
ros2 topic echo /right_foot/wrench

# View torso pose
ros2 topic echo /torso/pose
```

### Test 6: Send Joint Commands

Send commands to the robot:

```bash
# Example: Command all joints to zero position
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray \
  "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" \
  --once
```

## Next Steps

### 1. Customize the MuJoCo Model

The template model is located at:
```
src/simulation/humanoid_mujoco/models/humanoid.xml
```

Edit this file to:
- Add your actual robot meshes
- Adjust mass/inertia properties
- Fine-tune actuator gains
- Add additional sensors

After editing, rebuild:
```bash
colcon build --packages-select humanoid_mujoco
source install/setup.bash
```

### 2. Convert Your URDF to MJCF

If you have a URDF file:

```bash
ros2 run humanoid_mujoco urdf_to_mjcf.py \
  --urdf path/to/your/robot.urdf \
  --output models/humanoid.xml
```

**Note**: MuJoCo 2.1+ can include URDF files directly. The converter creates a wrapper MJCF.

### 3. Tune Physics Parameters

Edit `config/simulation_params.yaml`:

```yaml
mujoco_simulator:
  ros__parameters:
    timestep: 0.002          # Smaller = more accurate, slower
    solver_iterations: 50    # Higher = more accurate, slower
    realtime_factor: 1.0     # >1.0 = faster than realtime
```

### 4. Develop Balance Controller

See `config/balance_controller.yaml` for parameters.

The balance controller will subscribe to:
- `/imu/data` - For torso orientation
- `/left_foot/wrench` and `/right_foot/wrench` - For ground contact
- `/joint_states` - For current joint positions

And publish to:
- `/joint_commands` - Joint position commands

### 5. Develop Walking Controller

See `config/walking_controller.yaml` for gait parameters.

Launch walking demo:
```bash
ros2 launch humanoid_mujoco walking_demo.launch.py
```

## Performance Tips

### Speed Up Simulation

```yaml
# In config/simulation_params.yaml
realtime_factor: 2.0  # Run 2x faster than realtime
```

### Reduce Computation

```yaml
solver_iterations: 30  # Reduce from 50 (less accurate but faster)
timestep: 0.005       # Increase from 0.002 (larger steps, faster)
```

### Disable Visualization

```bash
ros2 launch humanoid_mujoco mujoco_simulation.launch.py \
  use_viewer:=false \
  use_rviz:=false
```

## Troubleshooting

### "MuJoCo module not found"

```bash
pip3 install mujoco
```

### "Model file not found"

Check the model path:
```bash
echo $(ros2 pkg prefix humanoid_mujoco)/share/humanoid_mujoco/models/humanoid.xml
ls -l $(ros2 pkg prefix humanoid_mujoco)/share/humanoid_mujoco/models/
```

### Robot Falls Immediately

The template model has approximate inertias. You need to:
1. Add proper mass/inertia values from your CAD
2. Tune actuator gains (kp values in the `<actuator>` section)
3. Start with higher gains for leg joints

Example actuator tuning in `humanoid.xml`:
```xml
<actuator>
  <!-- Increase kp for leg joints -->
  <position name="left_hip_pitch_motor" joint="left_hip_pitch" kp="1000"/>
  <position name="left_knee_pitch_motor" joint="left_knee_pitch" kp="1000"/>
</actuator>
```

### Poor Performance

MuJoCo is designed for speed. If performance is poor:
1. Check CPU usage - should be <20% on modern CPUs
2. Disable unnecessary sensors in the model
3. Reduce `publish_rate` in config
4. Use headless mode (no viewer)

## Comparison with Gazebo

If you're migrating from Gazebo:

| Aspect | Gazebo | MuJoCo |
|--------|--------|--------|
| Launch | `gazebo.launch.py` | `mujoco_simulation.launch.py` |
| Model Format | SDF/URDF | MJCF (can include URDF) |
| Viewer | Integrated GUI | Separate passive viewer |
| Plugin System | Gazebo plugins | Custom ROS nodes |
| Contact Dynamics | ODE/Bullet/DART | Convex optimization |
| Performance | Good | Excellent (5-10x faster) |

## Resources

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MJCF XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [MuJoCo Forum](https://github.com/deepmind/mujoco/discussions)
- [Example Models](https://github.com/deepmind/mujoco_menagerie)

## Support

For issues specific to this package:
1. Check the main README.md
2. Verify MuJoCo installation: `python3 -c "import mujoco; print(mujoco.__version__)"`
3. Check ROS 2 topics are publishing: `ros2 topic list`
4. Review log output: `ros2 launch humanoid_mujoco mujoco_simulation.launch.py`
