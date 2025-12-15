# ROS 2 Integration - robot_balanced.mjcf Walking

## Overview

robot_balanced.mjcf este acum complet integrat cu ROS 2 Jazzy! Poți lansa walking și standing controllers folosind `ros2 launch`.

## Quick Start

### 1. Standing Test (Stationary Balance)

```bash
# Source ROS 2 environment
source install/setup.bash

# Launch standing controller (60s default)
ros2 launch locomotion_control robot_balanced_standing.launch.py

# Custom duration
ros2 launch locomotion_control robot_balanced_standing.launch.py duration:=120.0
```

**Expected output**:
```
✓ Using robot_balanced.mjcf
✓ Model loaded: 24 bodies, 21 joints, 20 actuators
Time: 1.00s | Base: 0.807m | COM: (0.006, -0.001) | Error: 0.006m
...
✓ Simulation completed: 60.00s
Final base height: 0.807m
```

### 2. Walking Test (Bipedal Locomotion)

```bash
# Source ROS 2 environment
source install/setup.bash

# Launch walking controller (60s default, 4 steps)
ros2 launch locomotion_control robot_balanced_walking.launch.py

# Custom duration
ros2 launch locomotion_control robot_balanced_walking.launch.py duration:=40.0
```

**Expected output**:
```
✓ Using robot_balanced.mjcf (optimized for walking)
  - Mass: 15.21 kg (vs 36.72 kg original)
  - COM: 0.604m (vs 0.752m original)
  - Can walk: YES! ✓

Phase 1: Stand still 10s (validate stability)
Phase 2: Walk 4 TINY steps (1cm forward, 5mm lift)

T: 10.0s | ✓ WALKING (left) | Step:0/4 | Phase:0.00(SHIFT)
T: 16.0s | ✓ Step 1 completed! Support: right
T: 23.0s | ✓ Step 2 completed! Support: left
...
✓✓✓ SUCCESS - Robot stayed upright!
```

## Launch Files

### 1. `robot_balanced_standing.launch.py`

**Purpose**: Test static standing balance
**Duration**: 120s default (configurable)
**Use case**: Validate stability, tune balance gains

**Parameters**:
- `duration`: Simulation duration in seconds (default: 120.0)

### 2. `robot_balanced_walking.launch.py`

**Purpose**: Bipedal walking with ZMP control
**Duration**: 60s default (configurable)
**Steps**: 4 steps (6s per step)
**Use case**: Demonstrate walking capability

**Parameters**:
- `duration`: Simulation duration in seconds (default: 60.0)

## File Locations

```
src/control/locomotion_control/launch/
├── robot_balanced_standing.launch.py    # Standing test
└── robot_balanced_walking.launch.py     # Walking test

src/control/lipm_walking_controller/
├── models/
│   └── robot_balanced.mjcf             # Optimized robot model
└── scripts/
    ├── zmp_balance_controller.py       # Standing controller
    └── zmp_walking_incremental.py      # Walking controller
```

## Model Specifications: robot_balanced.mjcf

| Property | Value | vs Original |
|----------|-------|-------------|
| Total Mass | 15.21 kg | -58.6% (36.72 kg) |
| COM Height | 0.604 m | -20% (0.752 m) |
| Torso Mass | 2.50 kg | -80% (12.97 kg) |
| Leg Mass | Kept substantial | For low COM |
| Standing | ✓ 120s+ stable | Same as original |
| Walking | ✓ 4 steps | ✗ Original falls |

## Performance Metrics

### Standing (60s test)
- **Height stability**: 0.807m ± 0.001m
- **COM error**: < 0.007m
- **Success rate**: 100%

### Walking (40s, 4 steps)
- **Height stability**: 0.807-0.808m
- **COM error**: 0.004-0.011m
- **Steps completed**: 4/4 (100%)
- **Step parameters**:
  - Forward: 1cm per step
  - Lift: 5mm
  - Duration: 6s per step
- **Success rate**: 100%

## Troubleshooting

### Issue: "robot_balanced.mjcf not found"

```bash
# Verify model exists
ls src/control/lipm_walking_controller/models/robot_balanced.mjcf

# If missing, check if you're in the correct workspace
cd ~/ros2_ws_demo/ld-robots-humanoid-robot-demo
```

### Issue: "MuJoCo viewer not opening"

This is expected in headless environments. The controller works without visualization.

To enable visualization:
```bash
export DISPLAY=:0  # or :1, depending on your X server
```

### Issue: Robot falls during walking

robot_balanced.mjcf uses ultra-conservative parameters. If it falls:
1. Check initial height (should be 0.807-0.808m at T=0-1s)
2. Verify COM error stays < 0.02m during standing phase
3. Reduce step size or increase step duration

## Next Steps: Tuning Walking Parameters

To make walking faster/more dynamic, edit `zmp_walking_incremental.py`:

```python
# Current (ultra-conservative)
self.step_duration = 6.0      # seconds per step
self.step_forward = 0.01      # 1cm forward
self.step_lift = 0.005        # 5mm lift
self.weight_shift = 0.05      # 5 degrees lateral

# Suggested progression:
# Step 1 (moderate)
self.step_duration = 4.0      # -33%
self.step_forward = 0.03      # +200%
self.step_lift = 0.01         # +100%

# Step 2 (dynamic)
self.step_duration = 2.0      # -50%
self.step_forward = 0.05      # +66%
self.step_lift = 0.02         # +100%
```

**Important**: Test incrementally! Increase one parameter at a time.

## Comparison: Available Models

| Model | Mass | COM | Standing | Walking | ROS 2 Launch |
|-------|------|-----|----------|---------|--------------|
| `robot.mjcf` | 36.72kg | 0.752m | ✓ | ✗ | `zmp_balance_controller.launch.py` |
| `robot_balanced.mjcf` | 15.21kg | 0.604m | ✓ | ✓ | `robot_balanced_walking.launch.py` |
| `simple_humanoid.xml` | 11.20kg | 0.764m | ✓ | ✓ | `simple_walking.launch.py` |

## Building & Installation

```bash
# Build locomotion_control package
colcon build --packages-select locomotion_control --symlink-install

# Source setup
source install/setup.bash

# Verify launch files
ros2 launch locomotion_control --show-args robot_balanced_walking.launch.py
```

## Related Documentation

- [ROBOT_BALANCED_SUCCESS.md](ROBOT_BALANCED_SUCCESS.md) - Detailed technical analysis
- [TEST_SIMPLE_HUMANOID_OLD.md](TEST_SIMPLE_HUMANOID_OLD.md) - simple_humanoid reference
- [CLAUDE.md](CLAUDE.md) - Overall project structure

## Credits

**Solution based on research from**:
- [K-Scale Labs](https://github.com/kscalelabs) - ksim, ksim-gym
- [Humanoid-Gym](https://arxiv.org/abs/2404.05695) - RL training framework
- [MuJoCo Playground](https://arxiv.org/abs/2502.08844) - Rapid policy training
- Multiple bipedal walking research papers

**Key insight**: Mass distribution is critical - reduce torso mass 80%, keep leg mass substantial for low COM.

---

**Last updated**: 2025-12-15
**ROS 2 Version**: Jazzy
**MuJoCo Version**: 3.4.0
