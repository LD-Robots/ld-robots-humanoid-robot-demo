# Setup Guide

## Prerequisites

### ROS2 Jazzy Installation

```bash
# Update and install prerequisites
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add ROS2 repository
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop
```

### Additional Dependencies

```bash
# ROS2 development tools
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

# ROS2 Control
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gazebo-ros2-control

# Gazebo
sudo apt install -y ros-jazzy-ros-gz

# Python dependencies
sudo apt install -y \
  python3-pip \
  python3-opencv \
  python3-numpy

# Install Python packages
pip3 install opencv-python numpy
```

### Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

## Building the Workspace

```bash
# Navigate to workspace
cd ~/Documents/GitHub/ld-robots-humanoid-robot-demo

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Making Scripts Executable

```bash
# Make all Python scripts executable
chmod +x src/humanoid_perception/scripts/*.py
chmod +x src/humanoid_locomotion/scripts/*.py
chmod +x src/humanoid_manipulation/scripts/*.py
```

## Workspace Configuration

Add to your `~/.bashrc` for convenience:

```bash
# ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Humanoid workspace (adjust path as needed)
source ~/Documents/GitHub/ld-robots-humanoid-robot-demo/install/setup.bash

# Optional: Set default RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Verifying Installation

```bash
# Check if packages are found
ros2 pkg list | grep humanoid

# Expected output:
# humanoid_bringup
# humanoid_controllers
# humanoid_description
# humanoid_hardware
# humanoid_locomotion
# humanoid_manipulation
# humanoid_perception
# humanoid_simulation
```

## Next Steps

See [README.md](README.md) for usage instructions and examples.
