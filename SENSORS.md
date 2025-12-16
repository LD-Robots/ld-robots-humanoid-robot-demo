# Sensor Setup for Humanoid Robot

## Sensors Added ✅

### 1. **IMU (Inertial Measurement Unit)**
- **Location**: Base link (torso)
- **ROS Topic**: `/imu/data`
- **Frame**: `imu_link`
- **Update Rate**: 100 Hz
- **Data Published**:
  - Orientation (quaternion)
  - Angular velocity (rad/s)
  - Linear acceleration (m/s²)

### 2. **Force-Torque Sensors (Feet)**
- **Location**: Left and right ankle joints
- **ROS Topics**:
  - `/left_foot/ft_data`
  - `/right_foot/ft_data`
- **Update Rate**: 100 Hz
- **Data Published**:
  - Force (x, y, z) in Newtons
  - Torque (x, y, z) in Newton-meters

### 3. **Contact Sensors (Feet)**
- **Location**: Left and right foot links
- **ROS Topics**:
  - `/left_foot/contact`
  - `/right_foot/contact`
- **Update Rate**: 100 Hz
- **Data Published**:
  - Contact state (in contact or not)
  - Contact positions
  - Contact normals
  - Contact depths

---

## Files Modified

### URDF/Xacro Files:
1. ✅ Created [`src/robot_description/humanoid_description/xacro/sensors/imu.xacro`](src/robot_description/humanoid_description/xacro/sensors/imu.xacro)
2. ✅ Created [`src/robot_description/humanoid_description/xacro/sensors/force_torque.xacro`](src/robot_description/humanoid_description/xacro/sensors/force_torque.xacro)
3. ✅ Updated [`src/robot_description/humanoid_description/urdf/humanoid.urdf.xacro`](src/robot_description/humanoid_description/urdf/humanoid.urdf.xacro)
   - Added IMU sensor instantiation
   - Added F/T sensors to ankle joints
   - Added sensor interfaces to ros2_control

### Controller Configuration:
4. ✅ Updated [`src/simulation/humanoid_gazebo/config/controllers.yaml`](src/simulation/humanoid_gazebo/config/controllers.yaml)
   - Added `imu_sensor_broadcaster`
   - Added `force_torque_sensor_broadcaster`

---

## Testing the Sensors

### Build the Workspace
```bash
cd ~/Documents/GitHub/ld-robots-humanoid-robot-demo
colcon build --packages-select humanoid_description humanoid_gazebo
source install/setup.bash
```

### Launch Gazebo with Sensors
```bash
ros2 launch humanoid_gazebo gazebo.launch.py
```

### Check Sensor Topics
In another terminal:
```bash
# List all topics
ros2 topic list

# Should see:
# /imu/data
# /left_foot/ft_data
# /right_foot/ft_data
# /joint_states
```

### Monitor IMU Data
```bash
ros2 topic echo /imu/data
```

### Monitor Force-Torque Data
```bash
# Left foot
ros2 topic echo /left_foot/ft_data

# Right foot
ros2 topic echo /right_foot/ft_data
```

### Visualize in PlotJuggler
```bash
plotjuggler

# Then:
# 1. Streaming → ROS2 Topic Subscriber
# 2. Select topics: /imu/data, /left_foot/ft_data, /right_foot/ft_data
# 3. Drag data to plot area
```

---

## Sensor Data Format

### IMU (`sensor_msgs/Imu`)
```yaml
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: "imu_link"
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}  # Quaternion
angular_velocity: {x: 0.0, y: 0.0, z: 0.0}     # rad/s
linear_acceleration: {x: 0.0, y: 0.0, z: -9.81} # m/s² (gravity)
```

### Force-Torque (`geometry_msgs/WrenchStamped`)
```yaml
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: "left_ft_sensor_link"
wrench:
  force: {x: 0.0, y: 0.0, z: 490.0}    # N (half body weight ~50kg)
  torque: {x: 0.0, y: 0.0, z: 0.0}     # Nm
```

---

## Using Sensors for Balance Control

### Reading IMU in Python
```python
import rclpy
from sensor_msgs.msg import Imu

class BalanceController:
    def __init__(self):
        self.node = rclpy.create_node('balance_controller')
        self.imu_sub = self.node.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

    def imu_callback(self, msg):
        # Get orientation (roll, pitch, yaw)
        from tf_transformations import euler_from_quaternion
        roll, pitch, yaw = euler_from_quaternion([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])

        # If robot tips forward (pitch > 0.1 rad)
        if abs(pitch) > 0.1:
            print("Robot tipping! Pitch:", pitch)
            # Apply corrective ankle torque
```

### Reading Force-Torque in Python
```python
from geometry_msgs.msg import WrenchStamped

class BalanceController:
    def __init__(self):
        self.left_ft_sub = self.node.create_subscription(
            WrenchStamped, '/left_foot/ft_data', self.left_ft_callback, 10)
        self.right_ft_sub = self.node.create_subscription(
            WrenchStamped, '/right_foot/ft_data', self.right_ft_callback, 10)

    def left_ft_callback(self, msg):
        force_z = msg.wrench.force.z  # Vertical force

        # Check if foot is on ground (force > threshold)
        if force_z > 50.0:  # 50 N threshold
            print("Left foot on ground, force:", force_z)

        # Compute ZMP from CoP
        cop_x = -msg.wrench.torque.y / force_z
        cop_y = msg.wrench.torque.x / force_z
```

---

## Next Steps

### For Standing Balance:
1. ✅ Sensors configured
2. 🔜 Create balance controller node
3. 🔜 Compute CoM with Pinocchio
4. 🔜 Implement PID control on ankle/hip joints
5. 🔜 Test in Gazebo

### Required Packages (Install if needed):
```bash
sudo apt install -y \
    ros-jazzy-imu-sensor-broadcaster \
    ros-jazzy-force-torque-sensor-broadcaster \
    python3-transforms3d
```

---

## Troubleshooting

### Sensors not publishing?
```bash
# Check if controllers are loaded
ros2 control list_controllers

# Should see:
# imu_sensor_broadcaster[imu_sensor_broadcaster/IMUSensorBroadcaster] active
# force_torque_sensor_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
```

### IMU shows all zeros?
- Check Gazebo plugin is loaded: `gz topic -l | grep imu`
- Verify URDF compiled correctly: `xacro src/robot_description/humanoid_description/urdf/humanoid.urdf.xacro > /tmp/test.urdf`

### Force-torque shows no contact?
- Check if robot feet are touching ground in Gazebo
- Verify joint names match in URDF and sensor config
