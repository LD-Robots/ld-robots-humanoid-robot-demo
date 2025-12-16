# Pressure Mat System

## Overview

The pressure mat system provides real-time visualization of contact forces from the humanoid robot. A 1.0m x 0.6m pressure-sensing mat is placed on the ground in the Gazebo simulation, and contact forces are visualized as a heatmap.

## Features

- **Contact Detection**: Detects all contact points on the mat surface at 100 Hz
- **Real-time Heatmap**: Visualizes pressure distribution with color coding (blue=low, red=high)
- **3D Markers**: Optional RViz markers showing pressure intensity in 3D
- **Configurable Grid**: 20x12 grid cells (adjustable via parameters)
- **Pressure Decay**: Smooth decay for realistic visualization

## Architecture

```
┌─────────────────┐
│  Gazebo World   │
│  ┌───────────┐  │
│  │ Humanoid  │  │
│  │  Robot    │  │
│  └─────┬─────┘  │
│        │         │
│   ┌────▼─────┐  │
│   │Pressure  │  │
│   │   Mat    │  │  Contact Sensor (100 Hz)
│   └────┬─────┘  │
└────────┼────────┘
         │
    ┌────▼────────────────────────┐
    │  ros_gz_bridge              │
    │  /world/.../contact topic   │
    └────┬────────────────────────┘
         │
    ┌────▼──────────────────────┐
    │ pressure_mat_visualizer   │
    │  (Python Node)            │
    │  - Processes contacts     │
    │  - Creates heatmap        │
    │  - Generates markers      │
    └────┬─────────┬────────────┘
         │         │
         │         └───────────────┐
         │                         │
    ┌────▼──────┐         ┌───────▼─────────┐
    │ /pressure │         │ /pressure_mat/  │
    │ _mat/     │         │ markers         │
    │ heatmap   │         │ (MarkerArray)   │
    │ (Image)   │         └─────────────────┘
    └───────────┘
```

## Usage

### 1. Launch Simulation

The pressure mat is automatically included when you launch the Gazebo simulation:

```bash
# Source your workspace
source install/setup.bash

# Launch Gazebo with pressure mat
ros2 launch humanoid_gazebo gazebo.launch.py
```

### 2. View Heatmap

The heatmap is published as an image topic. You can view it using:

**Option A: rqt_image_view**
```bash
ros2 run rqt_image_view rqt_image_view
# Select topic: /pressure_mat/heatmap
```

**Option B: ROS 2 image viewer**
```bash
ros2 run image_view image_view --ros-args -r image:=/pressure_mat/heatmap
```

### 3. View 3D Markers in RViz (Optional)

```bash
# Launch RViz
rviz2

# Add MarkerArray display
# Set topic: /pressure_mat/markers
# Set Fixed Frame: world
```

### 4. Monitor Topics

```bash
# Check contact sensor data
ros2 topic echo /world/empty_world/model/pressure_mat/link/mat_base/sensor/pressure_mat_contact_sensor/contact

# Check heatmap output
ros2 topic info /pressure_mat/heatmap
ros2 topic hz /pressure_mat/heatmap

# Check marker output
ros2 topic info /pressure_mat/markers
```

## Configuration

The pressure mat visualizer accepts several parameters that can be adjusted in the launch file ([gazebo.launch.py:256-269](launch/gazebo.launch.py#L256-L269)):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `grid_size_x` | 20 | Number of cells in X direction |
| `grid_size_y` | 12 | Number of cells in Y direction |
| `mat_width` | 1.0 | Mat width in meters |
| `mat_height` | 0.6 | Mat height in meters |
| `update_rate` | 30.0 | Visualization update rate (Hz) |
| `force_scale` | 100.0 | Scale factor for force visualization |
| `decay_rate` | 0.9 | How fast pressure decays (0-1, 1=no decay) |

### Example: Adjust Parameters

Edit [gazebo.launch.py:256-269](launch/gazebo.launch.py#L256-L269):

```python
pressure_visualizer = Node(
    package='humanoid_gazebo',
    executable='pressure_mat_visualizer.py',
    name='pressure_mat_visualizer',
    output='screen',
    parameters=[
        {'grid_size_x': 40},      # Higher resolution
        {'grid_size_y': 24},
        {'force_scale': 50.0},    # More sensitive
        {'decay_rate': 0.95}      # Slower decay
    ]
)
```

## Files

### Models
- [models/pressure_mat/model.sdf](models/pressure_mat/model.sdf) - Pressure mat SDF model
- [models/pressure_mat/model.config](models/pressure_mat/model.config) - Model metadata

### Scripts
- [scripts/pressure_mat_visualizer.py](scripts/pressure_mat_visualizer.py) - Visualization node

### Launch
- [launch/gazebo.launch.py](launch/gazebo.launch.py) - Main launch file (includes pressure mat)

### World
- [worlds/empty.sdf](worlds/empty.sdf) - Gazebo world with pressure mat included

## Troubleshooting

### Issue: Pressure mat not visible in Gazebo

**Solution**: Check that the model path is set correctly:
```bash
echo $GZ_SIM_RESOURCE_PATH
# Should include: .../humanoid_gazebo/models
```

### Issue: No heatmap displayed

**Solution**:
1. Check that the visualizer node is running:
   ```bash
   ros2 node list | grep pressure_mat_visualizer
   ```

2. Verify contact sensor bridge is active:
   ```bash
   ros2 topic list | grep pressure_mat
   ```

3. Check for Python dependencies:
   ```bash
   python3 -c "import cv2; import numpy; print('Dependencies OK')"
   ```

### Issue: Heatmap shows no data

**Solution**:
1. Ensure robot is making contact with the mat (z=0)
2. Check contact sensor is receiving data:
   ```bash
   ros2 topic echo /world/empty_world/model/pressure_mat/link/mat_base/sensor/pressure_mat_contact_sensor/contact
   ```

3. Increase `decay_rate` to retain pressure longer
4. Decrease `force_scale` to make visualization more sensitive

### Issue: Heatmap is too sensitive/insensitive

**Solution**: Adjust the `force_scale` parameter:
- **Too sensitive** (always red): Increase `force_scale` (e.g., 200.0)
- **Not sensitive** (stays blue): Decrease `force_scale` (e.g., 50.0)

## Technical Details

### Mat Specifications
- **Dimensions**: 1.0m x 0.6m x 2mm
- **Position**: Centered at origin (0, 0, 0.001)
- **Material**: Semi-transparent blue-gray
- **Contact stiffness**: 1e6 N/m
- **Contact damping**: 100 N·s/m

### Sensor Specifications
- **Type**: Gazebo contact sensor
- **Update Rate**: 100 Hz
- **Collision**: Detects all collisions on mat surface

### Visualization Details
- **Grid Resolution**: 20 x 12 cells (configurable)
- **Color Map**: Jet colormap (blue → cyan → green → yellow → red)
- **Image Size**: 400 x 240 pixels (upscaled from grid)
- **Update Rate**: 30 Hz (configurable)

## Future Enhancements

Potential improvements:
1. **GUI Window**: PyQt5 standalone window with controls
2. **Recording**: Save pressure data to file for analysis
3. **Statistics**: Display CoP (Center of Pressure), total force, etc.
4. **Multiple Mats**: Support multiple pressure mats in different locations
5. **Calibration**: Add calibration routine for real hardware

## Related Documentation

- See [SENSORS.md](../../SENSORS.md) for robot sensor information
- See [TOOLS.md](../../TOOLS.md) for additional tools
