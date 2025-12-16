#!/bin/bash
# Humanoid Robot Dependencies Installation Script

echo "Installing additional ROS 2 Jazzy packages..."

# Navigation2 (for autonomous navigation)
sudo apt install -y \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-common \
    ros-jazzy-navigation2

# BehaviorTree (for high-level task planning)
sudo apt install -y \
    ros-jazzy-behaviortree-cpp-v3

# Additional control packages
sudo apt install -y \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-ros2-controllers

# Gazebo-related packages
sudo apt install -y \
    ros-jazzy-gazebo-ros2-control \
    ros-jazzy-gazebo-ros-pkgs

# Visualization and tools
sudo apt install -y \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-joint-state-publisher-gui

# URDF/xacro tools
sudo apt install -y \
    ros-jazzy-xacro \
    liburdfdom-tools

# Pinocchio (dynamics library) - already installed via ros-jazzy-pinocchio
# Note: ros-jazzy-pinocchio includes Python bindings

# Eigen (linear algebra library)
sudo apt install -y \
    libeigen3-dev

# Python packages for development
# IMPORTANT: Downgrade numpy to 1.x for ROS 2 Jazzy compatibility
pip3 install --user \
    "numpy<2" \
    scipy \
    matplotlib \
    transforms3d

echo ""
echo "⚠️  NOTE: NumPy downgraded to 1.x for ROS 2 Jazzy compatibility"

echo "✅ Installation complete!"
echo ""
echo "To verify installations, run:"
echo "  ros2 pkg list | grep -E '(moveit|pinocchio|nav2)'"
echo ""
echo "Optional specialized tools for humanoid development:"
echo ""
echo "1. ADVANCED VISUALIZATION:"
echo "   sudo snap install plotjuggler       # Already installed! Real-time plotting"
echo ""
echo "2. TRAJECTORY OPTIMIZATION:"
echo "   # TOWR (Trajectory Optimizer for Walking Robots)"
echo "   cd ~/Documents && git clone https://github.com/ethz-adrl/towr.git"
echo ""
echo "3. WHOLE-BODY CONTROL:"
echo "   # Crocoddyl (optimal control library)"
echo "   sudo apt install robotpkg-py312-crocoddyl"
echo "   # OR compile from source: https://github.com/loco-3d/crocoddyl"
echo ""
echo "4. KINEMATICS/DYNAMICS:"
echo "   # KDL (Kinematics and Dynamics Library) - alternative to Pinocchio"
echo "   sudo apt install ros-jazzy-kdl-parser"
echo ""
echo "5. REAL-TIME TOOLS:"
echo "   # For real-time performance analysis"
echo "   sudo apt install rt-tests stress-ng"
echo ""
echo "6. MOTION CAPTURE INTEGRATION:"
echo "   # For mocap-based teleoperation"
echo "   sudo apt install ros-jazzy-mocap-optitrack"
echo ""
echo "7. FOOTSTEP PLANNING:"
echo "   # Grid-based footstep planner"
echo "   sudo apt install ros-jazzy-gridmap-core"
