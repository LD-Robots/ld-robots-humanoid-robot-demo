# Humanoid Robot Development Tools

## ✅ Already Installed (Core Tools)

### Simulation & Visualization
- **Gazebo (gz)** - Physics simulation
- **RViz2** - 3D robot visualization
- **PlotJuggler** - Real-time data plotting
- **rqt** - Qt-based GUI tools

### Control & Planning
- **MoveIt** - Motion planning framework
- **ros2_control** - Controller framework
- **Joint Trajectory Controller** - Trajectory execution
- **Joint State Broadcaster** - Joint state publishing
- **Pinocchio** - Fast rigid body dynamics

### Navigation & Behaviors
- **Nav2** - Navigation stack
- **BehaviorTree.CPP (v3 & v4)** - Behavior trees

### Development
- **xacro** - XML macro language for URDF
- **urdfdom-tools** - URDF validation
- **Eigen3** - Linear algebra library

---

## 🔧 Recommended Additional Tools

### 1. **Debugging & Analysis Tools**
```bash
# ros2 doctor - ROS 2 system checker
sudo apt install ros-jazzy-ros2doctor

# TF tools for coordinate frame debugging
sudo apt install ros-jazzy-tf2-tools

# Robot state publisher (if not installed)
sudo apt install ros-jazzy-robot-state-publisher
```

**Use cases:**
- Debug TF frame issues
- Monitor system health
- Visualize TF tree

---

### 2. **Advanced Controllers**
```bash
# Admittance controller (for compliant manipulation)
sudo apt install ros-jazzy-admittance-controller

# Force torque sensor controller
sudo apt install ros-jazzy-force-torque-sensor-broadcaster

# IMU sensor broadcaster
sudo apt install ros-jazzy-imu-sensor-broadcaster

# Effort controllers (torque control)
sudo apt install ros-jazzy-effort-controllers
```

**Use cases:**
- Balance control with force/torque feedback
- IMU-based stabilization
- Compliant interaction

---

### 3. **Trajectory Optimization (Advanced Walking)**
```bash
# TOWR - Trajectory Optimizer for Walking Robots
# (Must compile from source)
cd ~/Documents
git clone https://github.com/ethz-adrl/towr.git
cd towr
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

**Use cases:**
- Optimal footstep trajectories
- Dynamic walking gaits
- Terrain adaptation

---

### 4. **Whole-Body Optimal Control**
```bash
# Crocoddyl - Contact RObot COntrol by Differential DYnamic programming Library
# Option 1: From robotpkg (if available)
sudo apt install robotpkg-py312-crocoddyl

# Option 2: From source (recommended)
cd ~/Documents
git clone --recursive https://github.com/loco-3d/crocoddyl.git
cd crocoddyl
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=ON
make -j$(nproc)
sudo make install
```

**Use cases:**
- Model Predictive Control (MPC)
- Whole-body trajectory optimization
- Contact-implicit planning

---

### 5. **Grid Mapping for Footstep Planning**
```bash
# GridMap - Universal grid map library
sudo apt install ros-jazzy-grid-map-core \
                 ros-jazzy-grid-map-ros \
                 ros-jazzy-grid-map-msgs
```

**Use cases:**
- Terrain height maps
- Traversability analysis
- Footstep placement

---

### 6. **Real-Time Performance Tools**
```bash
# RT tests and CPU stress testing
sudo apt install rt-tests stress-ng

# cyclictest for latency testing
sudo apt install libnuma-dev
```

**Use cases:**
- Measure control loop latency
- Verify real-time performance
- Stress test system

---

### 7. **Motion Capture Integration (Optional)**
```bash
# OptiTrack mocap driver
sudo apt install ros-jazzy-mocap-optitrack

# VICON mocap driver
sudo apt install ros-jazzy-vicon-bridge
```

**Use cases:**
- Motion retargeting
- VR teleoperation
- Ground truth localization

---

### 8. **Computer Vision (For Perception)**
```bash
# OpenCV with ROS
sudo apt install ros-jazzy-cv-bridge \
                 ros-jazzy-image-transport \
                 ros-jazzy-compressed-image-transport

# PCL (Point Cloud Library)
sudo apt install ros-jazzy-pcl-ros \
                 ros-jazzy-pcl-conversions

# Vision pipelines
sudo apt install ros-jazzy-vision-opencv \
                 ros-jazzy-image-pipeline
```

**Use cases:**
- Object detection
- Visual servoing
- Scene understanding

---

### 9. **Advanced URDF/SDF Tools**
```bash
# Mesh processing
sudo apt install meshlab blender

# COLLADA/STL tools
sudo apt install assimp-utils

# Gazebo model editor
# (Already included with Gazebo)
```

**Use cases:**
- Create/edit robot meshes
- Optimize collision geometries
- Generate URDFs

---

### 10. **Data Recording & Playback**
```bash
# rosbag2 tools (should be installed already)
sudo apt install ros-jazzy-rosbag2 \
                 ros-jazzy-rosbag2-storage-default-plugins

# ros2bag Python API
pip3 install rosbags
```

**Use cases:**
- Record sensor data
- Replay experiments
- Dataset generation

---

## 📊 Development Workflow Tools

### Python Libraries for Robotics
```bash
pip3 install --user \
    transforms3d \      # 3D transformations
    spatialmath-python \  # Spatial math library
    roboticstoolbox-python \  # Robotics algorithms
    pybullet \          # Physics simulation (alternative)
    trimesh              # Mesh processing
```

### C++ Libraries
```bash
# Additional Eigen utilities
sudo apt install libeigen3-dev

# Boost (usually already installed)
sudo apt install libboost-all-dev

# YAML-CPP for config parsing
sudo apt install libyaml-cpp-dev

# spdlog for fast logging
sudo apt install libspdlog-dev
```

---

## 🎯 Recommended Installation Order (For Your Project)

### **Phase 1: Immediate (Standing/Walking)**
```bash
# Already have these! Just verify:
ros2 pkg list | grep -E '(moveit|pinocchio|controller)'
```

### **Phase 2: Balance & Control (Week 2-3)**
```bash
sudo apt install \
    ros-jazzy-imu-sensor-broadcaster \
    ros-jazzy-force-torque-sensor-broadcaster \
    ros-jazzy-effort-controllers \
    ros-jazzy-tf2-tools
```

### **Phase 3: Advanced Walking (Week 4-5)**
```bash
# TOWR (compile from source - see above)
# Grid maps for footstep planning
sudo apt install ros-jazzy-grid-map-core ros-jazzy-grid-map-ros
```

### **Phase 4: Perception (Week 6+)**
```bash
sudo apt install \
    ros-jazzy-cv-bridge \
    ros-jazzy-pcl-ros \
    ros-jazzy-image-pipeline
```

---

## 🚀 Quick Install Script for Phase 2

Run this when you're ready for balance control:

```bash
#!/bin/bash
echo "Installing Phase 2 tools for balance & control..."

sudo apt install -y \
    ros-jazzy-imu-sensor-broadcaster \
    ros-jazzy-force-torque-sensor-broadcaster \
    ros-jazzy-effort-controllers \
    ros-jazzy-tf2-tools \
    ros-jazzy-ros2doctor \
    ros-jazzy-grid-map-core \
    ros-jazzy-grid-map-ros \
    ros-jazzy-admittance-controller

pip3 install --user \
    transforms3d \
    spatialmath-python

echo "✅ Phase 2 tools installed!"
```

---

## 📝 Summary

**You currently have everything needed for:**
- ✅ Basic simulation (Gazebo)
- ✅ Visualization (RViz2, PlotJuggler)
- ✅ Motion planning (MoveIt)
- ✅ Control (ros2_control, joint controllers)
- ✅ Kinematics/Dynamics (Pinocchio)

**Install next (when needed):**
- 🔜 IMU/FT sensor broadcasters (for balance)
- 🔜 TF tools (for debugging)
- 🔜 Grid maps (for footstep planning)
- 🔧 TOWR (for advanced walking - optional)
- 🔧 Crocoddyl (for MPC - optional)

**My recommendation:** Don't install everything now. Install tools as you need them during development. This keeps your system clean and you'll understand each tool better when you actually use it.
