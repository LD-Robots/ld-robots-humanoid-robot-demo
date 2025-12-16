# Four-Point Contact Integration Guide

## Overview

This guide shows how to add 4-point contact to each foot, allowing Gazebo to stabilize on all 4 corners instead of just 3 points.

## How It Works

Instead of a single rigid foot collision, we create **4 small spherical "bumpers"** at each corner:
- **Front Left (FL)** - Red sphere
- **Front Right (FR)** - Orange sphere
- **Back Left (BL)** - Blue sphere
- **Back Right (BR)** - Sky blue sphere

Each sphere has:
- **Soft contact parameters** (`kp=5000` vs normal `1000000`) - allows 5mm deformation
- **Independent collision** - can make contact separately
- **Contact sensor** - publishes contact state to ROS topic

## Files Created

1. **`four_point_foot_contact.xacro`** - Xacro macro for 4-point contact system
2. **`robot_with_4point_feet.urdf.xacro`** - Example integration template

## Integration Steps

### Step 1: Disable Main Foot Collision

In your `robot.urdf`, find the foot links and **disable their collision**:

```xml
<!-- Around line 566 - LEFT FOOT -->
<link name="LFootBushing_GPF_1517_12">
  <visual>
    <!-- Keep visual as-is -->
  </visual>

  <!-- DISABLE collision by commenting it out -->
  <!--
  <collision name="LFootBushing_GPF_1517_12.collision">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry name="LFootBushing_GPF_1517_12_collision_geometry">
      <mesh filename="package://humanoid_description/meshes/collision/LFootBushing_GPF_1517_12.collision.stl"/>
    </geometry>
  </collision>
  -->

  <!-- Keep inertial as-is -->
</link>

<!-- Around line 612 - RIGHT FOOT -->
<link name="RFootBushing_GPF_1517_12">
  <visual>
    <!-- Keep visual -->
  </visual>

  <!-- DISABLE collision -->
  <!--
  <collision name="RFootBushing_GPF_1517_12.collision">
    ...
  </collision>
  -->

  <!-- Keep inertial -->
</link>
```

### Step 2: Remove Soft Contact Parameters from Main Foot

Remove or comment out the Gazebo blocks around lines 566-589 and 636-658:

```xml
<!-- REMOVE OR COMMENT OUT:
<gazebo reference="LFootBushing_GPF_1517_12">
  <max_contacts>20</max_contacts>
  <collision>
    <surface>
      ...
    </surface>
  </collision>
</gazebo>
-->
```

### Step 3: Add Four-Point Contact Macro

At the end of your `robot.urdf` (before `</robot>`), add:

```xml
  <!-- Include the four-point contact macro -->
  <!-- Note: If using .urdf, you need to convert to .urdf.xacro first -->
  <!-- Or manually copy the expanded macro (8 links + 8 joints + gazebo tags) -->

  <!-- LEFT FOOT - 4 CONTACT POINTS -->

  <!-- Front Left -->
  <link name="left_foot_contact_fl">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000001" ixy="0" ixz="0"
               iyy="0.000001" iyz="0"
               izz="0.000001"/>
    </inertial>
    <visual>
      <geometry>
        <sphere radius="0.012"/>
      </geometry>
      <material name="contact_fl_material">
        <color rgba="1.0 0.2 0.2 0.9"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.012"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_foot_contact_fl_joint" type="fixed">
    <parent link="LFootBushing_GPF_1517_12"/>
    <child link="left_foot_contact_fl"/>
    <origin xyz="0.060 0.050 -0.035" rpy="0 0 0"/>
  </joint>

  <!-- Front Right -->
  <link name="left_foot_contact_fr">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000001" ixy="0" ixz="0"
               iyy="0.000001" iyz="0"
               izz="0.000001"/>
    </inertial>
    <visual>
      <geometry>
        <sphere radius="0.012"/>
      </geometry>
      <material name="contact_fr_material">
        <color rgba="1.0 0.4 0.2 0.9"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.012"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_foot_contact_fr_joint" type="fixed">
    <parent link="LFootBushing_GPF_1517_12"/>
    <child link="left_foot_contact_fr"/>
    <origin xyz="0.060 -0.050 -0.035" rpy="0 0 0"/>
  </joint>

  <!-- Back Left -->
  <link name="left_foot_contact_bl">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000001" ixy="0" ixz="0"
               iyy="0.000001" iyz="0"
               izz="0.000001"/>
    </inertial>
    <visual>
      <geometry>
        <sphere radius="0.012"/>
      </geometry>
      <material name="contact_bl_material">
        <color rgba="0.2 0.2 1.0 0.9"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.012"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_foot_contact_bl_joint" type="fixed">
    <parent link="LFootBushing_GPF_1517_12"/>
    <child link="left_foot_contact_bl"/>
    <origin xyz="-0.100 0.050 -0.035" rpy="0 0 0"/>
  </joint>

  <!-- Back Right -->
  <link name="left_foot_contact_br">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000001" ixy="0" ixz="0"
               iyy="0.000001" iyz="0"
               izz="0.000001"/>
    </inertial>
    <visual>
      <geometry>
        <sphere radius="0.012"/>
      </geometry>
      <material name="contact_br_material">
        <color rgba="0.2 0.4 1.0 0.9"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.012"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_foot_contact_br_joint" type="fixed">
    <parent link="LFootBushing_GPF_1517_12"/>
    <child link="left_foot_contact_br"/>
    <origin xyz="-0.100 -0.050 -0.035" rpy="0 0 0"/>
  </joint>

  <!-- Repeat for RIGHT FOOT with prefix "right" and parent "RFootBushing_GPF_1517_12" -->
  <!-- ... (same structure, 4 more links + 4 more joints) -->

  <!-- GAZEBO SOFT CONTACT PARAMETERS -->
  <gazebo reference="left_foot_contact_fl">
    <max_contacts>4</max_contacts>
    <collision>
      <surface>
        <contact>
          <ode>
            <kp>5000.0</kp>
            <kd>50.0</kd>
            <max_vel>0.1</max_vel>
            <min_depth>0.005</min_depth>
          </ode>
        </contact>
        <friction>
          <ode>
            <mu>1.5</mu>
            <mu2>1.5</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
    <material>Gazebo/Red</material>
  </gazebo>

  <!-- Repeat gazebo block for fr, bl, br -->
  <!-- ... -->
```

## Tuning Parameters

### Contact Stiffness (`kp`)
- **Default**: `5000.0` - Soft, allows 5mm compression
- **Softer**: `1000.0` - More compression, more stable on uneven ground
- **Stiffer**: `10000.0` - Less compression, faster response

### Contact Damping (`kd`)
- **Default**: `50.0` - Moderate damping
- **Lower**: `10.0` - Bouncier, less energy dissipation
- **Higher**: `100.0` - More damped, less oscillation

### Penetration Depth (`min_depth`)
- **Default**: `0.005` (5mm)
- Adjust based on how much "give" you want

### Sphere Positions
Adjust the XYZ offsets in the joint origins to match your actual foot geometry:
- **X axis**: Front (+) to back (-)
- **Y axis**: Left (+) to right (-)
- **Z axis**: Down (-)

## Testing

1. **Visualize in RViz**:
   ```bash
   ros2 launch humanoid_description display.launch.py
   ```
   You should see 4 colored spheres at each foot corner.

2. **Test in Gazebo**:
   ```bash
   ros2 launch humanoid_gazebo gazebo.launch.py
   ```
   The robot should now stabilize on all 4 points per foot.

3. **Monitor contacts**:
   ```bash
   ros2 topic list | grep contact
   ros2 topic echo /left_foot/contact_fl
   ```

## Expected Behavior

- **Standing**: All 8 spheres (4 per foot) should be in contact
- **Balancing**: Contact sensors show which corners are loaded
- **Stability**: Robot should be more stable on flat surfaces
- **Uneven terrain**: Individual spheres can compress independently

## Troubleshooting

### Robot sinks into ground
- Increase `kp` (make springs stiffer)
- Increase sphere `radius`
- Check that main foot collision is disabled

### Robot bounces/oscillates
- Increase `kd` (more damping)
- Reduce `max_vel`

### Only 3 points contact
- Make contacts softer (reduce `kp`)
- Increase `min_depth`
- Check sphere positions are coplanar

### Robot tips over easily
- Increase friction (`mu`, `mu2`)
- Widen foot contact positions
- Check center of mass is between contact points
