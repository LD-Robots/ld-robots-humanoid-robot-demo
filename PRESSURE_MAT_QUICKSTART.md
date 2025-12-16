# Pressure Mat Quick Start Guide

## What is it?

A **pressure-sensing floor mat** in Gazebo that visualizes where and how hard your humanoid robot is making contact with the ground. Perfect for debugging balance, gait, and contact dynamics!

## Quick Start (3 steps)

### 1. Launch Simulation
```bash
source install/setup.bash
ros2 launch humanoid_gazebo gazebo.launch.py
```

### 2. View the Heatmap
Open a new terminal:
```bash
ros2 run rqt_image_view rqt_image_view
```
Then select topic: `/pressure_mat/heatmap`

### 3. Watch the Magic!
- **Blue** = No pressure
- **Green/Yellow** = Medium pressure
- **Red** = High pressure

The robot's feet will show up as colored spots on the heatmap!

## What You'll See

```
┌─────────────────────────────┐
│     Pressure Heatmap        │
│  ┌───────────────────────┐  │
│  │         🟦🟦🟦        │  │  ← No contact (blue)
│  │    🔴🔴   🔴🔴      │  │  ← Left/right feet (red = high pressure)
│  │    🟡🟡   🟡🟡      │  │
│  │         🟦🟦🟦        │  │
│  └───────────────────────┘  │
│  Max: 450.23N               │
└─────────────────────────────┘
```

## Mat Specifications

- **Size**: 1.0m x 0.6m (centered at robot spawn point)
- **Grid**: 100 x 60 cells (very dense for detailed foot visualization)
- **Update Rate**: 30 Hz visualization
- **Sensor Rate**: 100 Hz contact detection
- **Spreading**: radius=8 cells (creates foot-shaped pressure zones)

## Advanced Usage

### View in RViz (3D visualization)
```bash
rviz2
# Add MarkerArray display
# Topic: /pressure_mat/markers
# Fixed Frame: world
```

### Monitor Raw Data
```bash
# Contact sensor data
ros2 topic echo /world/empty_world/model/pressure_mat/link/mat_base/sensor/pressure_mat_contact_sensor/contact

# Check update rate
ros2 topic hz /pressure_mat/heatmap
```

### Adjust Sensitivity

Edit `src/simulation/humanoid_gazebo/launch/gazebo.launch.py` (lines 261-268):

```python
parameters=[
    {'force_scale': 50.0},    # Lower = more sensitive (default: 100.0)
    {'decay_rate': 0.95}      # Higher = slower decay (default: 0.9)
]
```

Then rebuild:
```bash
colcon build --packages-select humanoid_gazebo
source install/setup.bash
```

## Files Created

```
src/simulation/humanoid_gazebo/
├── models/pressure_mat/          # Pressure mat model
│   ├── model.config
│   └── model.sdf
├── scripts/
│   └── pressure_mat_visualizer.py  # Visualization node
├── worlds/
│   └── empty.sdf                   # Updated world (includes mat)
├── launch/
│   └── gazebo.launch.py            # Updated launch file
└── README_PRESSURE_MAT.md          # Full documentation
```

## Troubleshooting

### No heatmap showing?
```bash
# Check node is running
ros2 node list | grep pressure_mat_visualizer

# Check dependencies
python3 -c "import cv2, numpy; print('OK')"
```

### Mat not visible in Gazebo?
The mat is semi-transparent and only 2mm thick. Look for a blue-gray rectangle on the ground at the origin.

### Too sensitive / not sensitive?
- **Too sensitive**: Increase `force_scale` in launch file
- **Not sensitive**: Decrease `force_scale` in launch file

## What's Next?

1. **Test it**: Make your robot stand/walk and watch the pressure distribution
2. **Record data**: Use rosbag to record pressure data for analysis
3. **Use for balance**: Monitor CoP (Center of Pressure) for balance control
4. **Debug gait**: Visualize foot contact timing during walking

## Full Documentation

See [README_PRESSURE_MAT.md](src/simulation/humanoid_gazebo/README_PRESSURE_MAT.md) for complete technical details.

---

**Pro Tip**: The pressure visualization has smooth decay, so you can see where the robot *was* standing even after it moves. Adjust `decay_rate` to control this!
