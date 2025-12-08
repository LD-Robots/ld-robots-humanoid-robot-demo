# Commented Out Dependencies

This file tracks all dependencies that have been commented out in package.xml and CMakeLists.txt files to allow the workspace to build without external dependencies.

## Overview

All 61 packages build successfully with these dependencies commented out. When you're ready to use these features, install the required packages and uncomment the relevant lines.

---

## 1. BehaviorTree.CPP (4 packages)

### Required Package
```bash
sudo apt install ros-jazzy-behaviortree-cpp-v3
```

### Affected Packages

#### `behavior_trees`
**Files**:
- `src/behavior/behavior_trees/package.xml` (line 13)
- `src/behavior/behavior_trees/CMakeLists.txt` (line 11)

**Commented lines**:
```xml
<!-- <depend>behaviortree_cpp_v3</depend> -->
```
```cmake
# find_package(behaviortree_cpp_v3 REQUIRED)
```

#### `manipulation_behaviors`
**Files**:
- `src/behavior/manipulation_behaviors/package.xml`
- `src/behavior/manipulation_behaviors/CMakeLists.txt`

**Commented lines**:
```xml
<!-- <depend>behaviortree_cpp_v3</depend> -->
```
```cmake
# find_package(behaviortree_cpp_v3 REQUIRED)
```

#### `locomotion_behaviors`
**Files**:
- `src/behavior/locomotion_behaviors/package.xml`
- `src/behavior/locomotion_behaviors/CMakeLists.txt`

**Commented lines**:
```xml
<!-- <depend>behaviortree_cpp_v3</depend> -->
```
```cmake
# find_package(behaviortree_cpp_v3 REQUIRED)
```

#### `interaction_behaviors`
**Files**:
- `src/behavior/interaction_behaviors/package.xml`
- `src/behavior/interaction_behaviors/CMakeLists.txt`

**Commented lines**:
```xml
<!-- <depend>behaviortree_cpp_v3</depend> -->
```
```cmake
# find_package(behaviortree_cpp_v3 REQUIRED)
```

---

## 2. Navigation2 (3 packages)

### Required Packages
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-common
```

### Affected Packages

#### `humanoid_navigation`
**Files**:
- `src/navigation/humanoid_navigation/package.xml`
- `src/navigation/humanoid_navigation/CMakeLists.txt`

**Commented lines**:
```xml
<!-- <depend>nav2_common</depend> -->
<!-- <depend>nav2_msgs</depend> -->
```
```cmake
# find_package(nav2_common REQUIRED)
# find_package(nav2_msgs REQUIRED)
```

#### `footstep_planning`
**Files**:
- `src/navigation/footstep_planning/package.xml`
- `src/navigation/footstep_planning/CMakeLists.txt`

**Commented lines**:
```xml
<!-- <depend>nav2_common</depend> -->
<!-- <depend>nav2_msgs</depend> -->
```
```cmake
# find_package(nav2_common REQUIRED)
# find_package(nav2_msgs REQUIRED)
```

#### `obstacle_avoidance`
**Files**:
- `src/navigation/obstacle_avoidance/package.xml`
- `src/navigation/obstacle_avoidance/CMakeLists.txt`

**Commented lines**:
```xml
<!-- <depend>nav2_common</depend> -->
<!-- <depend>nav2_msgs</depend> -->
```
```cmake
# find_package(nav2_common REQUIRED)
# find_package(nav2_msgs REQUIRED)
```

---

## 3. Gazebo (2 packages)

### Required Packages
```bash
sudo apt install ros-jazzy-gazebo-ros-pkgs
```

### Affected Packages

#### `humanoid_gazebo`
**Files**:
- `src/simulation/humanoid_gazebo/package.xml`
- `src/simulation/humanoid_gazebo/CMakeLists.txt`

**Commented lines**:
```xml
<!-- <depend>gazebo_ros</depend> -->
<!-- <depend>gazebo_plugins</depend> -->
```
```cmake
# find_package(gazebo_ros REQUIRED)
# find_package(gazebo_plugins REQUIRED)
```

#### `simulation_tools`
**Files**:
- `src/simulation/simulation_tools/package.xml`
- `src/simulation/simulation_tools/CMakeLists.txt`

**Commented lines**:
```xml
<!-- <depend>gazebo_ros</depend> -->
<!-- <depend>gazebo_plugins</depend> -->
```
```cmake
# find_package(gazebo_ros REQUIRED)
# find_package(gazebo_plugins REQUIRED)
```

---

## 4. Robot Localization (1 package)

### Required Package
```bash
sudo apt install ros-jazzy-robot-localization
```

### Affected Package

#### `state_estimation`
**Files**:
- `src/localization/state_estimation/package.xml`
- `src/localization/state_estimation/CMakeLists.txt`

**Commented lines**:
```xml
<!-- <depend>robot_localization</depend> -->
```
```cmake
# find_package(robot_localization REQUIRED)
```

---

## Quick Install All Dependencies

To install all commented dependencies at once:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-behaviortree-cpp-v3 \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-common \
  ros-jazzy-gazebo-ros-pkgs \
  ros-jazzy-robot-localization
```

---

## How to Uncomment

### Option 1: Manual Uncommenting
For each package, edit both files:

1. **package.xml**: Remove `<!-- -->` around the dependency
   ```xml
   <!-- Before -->
   <!-- <depend>behaviortree_cpp_v3</depend> -->

   <!-- After -->
   <depend>behaviortree_cpp_v3</depend>
   ```

2. **CMakeLists.txt**: Remove `#` from find_package line
   ```cmake
   # Before
   # find_package(behaviortree_cpp_v3 REQUIRED)

   # After
   find_package(behaviortree_cpp_v3 REQUIRED)
   ```

### Option 2: Batch Uncommenting with sed

After installing the dependencies, you can use these commands to uncomment:

```bash
# Uncomment BehaviorTree dependencies
for pkg in behavior_trees manipulation_behaviors locomotion_behaviors interaction_behaviors; do
  sed -i 's/<!-- <depend>behaviortree_cpp_v3<\/depend> -->/<depend>behaviortree_cpp_v3<\/depend>/' src/behavior/$pkg/package.xml
  sed -i 's/# find_package(behaviortree_cpp_v3 REQUIRED)/find_package(behaviortree_cpp_v3 REQUIRED)/' src/behavior/$pkg/CMakeLists.txt
done

# Uncomment Nav2 dependencies
for pkg in humanoid_navigation footstep_planning obstacle_avoidance; do
  sed -i 's/<!-- <depend>nav2_common<\/depend> -->/<depend>nav2_common<\/depend>/' src/navigation/$pkg/package.xml
  sed -i 's/<!-- <depend>nav2_msgs<\/depend> -->/<depend>nav2_msgs<\/depend>/' src/navigation/$pkg/package.xml
  sed -i 's/# find_package(nav2_common REQUIRED)/find_package(nav2_common REQUIRED)/' src/navigation/$pkg/CMakeLists.txt
  sed -i 's/# find_package(nav2_msgs REQUIRED)/find_package(nav2_msgs REQUIRED)/' src/navigation/$pkg/CMakeLists.txt
done

# Uncomment Gazebo dependencies
for pkg in humanoid_gazebo simulation_tools; do
  sed -i 's/<!-- <depend>gazebo_ros<\/depend> -->/<depend>gazebo_ros<\/depend>/' src/simulation/$pkg/package.xml
  sed -i 's/<!-- <depend>gazebo_plugins<\/depend> -->/<depend>gazebo_plugins<\/depend>/' src/simulation/$pkg/package.xml
  sed -i 's/# find_package(gazebo_ros REQUIRED)/find_package(gazebo_ros REQUIRED)/' src/simulation/$pkg/CMakeLists.txt
  sed -i 's/# find_package(gazebo_plugins REQUIRED)/find_package(gazebo_plugins REQUIRED)/' src/simulation/$pkg/CMakeLists.txt
done

# Uncomment robot_localization
sed -i 's/<!-- <depend>robot_localization<\/depend> -->/<depend>robot_localization<\/depend>/' src/localization/state_estimation/package.xml
sed -i 's/# find_package(robot_localization REQUIRED)/find_package(robot_localization REQUIRED)/' src/localization/state_estimation/CMakeLists.txt

# Rebuild workspace
colcon build
```

---

## Verification

After uncommenting and rebuilding, verify all packages build:

```bash
colcon build --symlink-install
```

If successful, you should see:
```
Summary: 61 packages finished [X.XXs]
```

---

## Notes

- **Current Status**: All 61 packages build successfully with dependencies commented
- **No Functionality Loss**: Packages compile but won't have full functionality until dependencies are installed and uncommented
- **Development Priority**: Based on the 2-month timeline, install dependencies as needed:
  - **Phase 1 (Weeks 1-3)**: No external dependencies needed
  - **Phase 2 (Weeks 4-6)**: Install Gazebo + robot_localization
  - **Phase 3 (Weeks 7-8)**: Install remaining based on feature priority

---

## Related Files

- [DEPENDENCIES.md](DEPENDENCIES.md) - Complete dependency analysis
- [CLAUDE.md](CLAUDE.md) - Project documentation
- [README.md](README.md) - Project overview
