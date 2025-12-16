#!/usr/bin/env python3
"""
URDF to MuJoCo MJCF converter for humanoid robot.
This script converts the humanoid URDF to MuJoCo's XML format (MJCF).
"""

import os
import sys
import argparse
from pathlib import Path

def convert_urdf_to_mjcf(urdf_path: str, output_path: str):
    """
    Convert URDF to MuJoCo MJCF format.

    Note: This is a template. For actual conversion, use:
    1. MuJoCo's compile functionality: mujoco.MjModel.from_xml_path()
    2. Or use third-party tools like dm_control's mjcf library
    3. Or use the mujoco_menagerie converter
    """
    print(f"Converting URDF from: {urdf_path}")
    print(f"Output MJCF to: {output_path}")

    # Check if MuJoCo Python bindings are available
    try:
        import mujoco
        print(f"MuJoCo version: {mujoco.__version__}")

        # MuJoCo can directly load URDF files
        # We'll create a wrapper MJCF that includes the URDF
        mjcf_content = generate_mjcf_wrapper(urdf_path)

        with open(output_path, 'w') as f:
            f.write(mjcf_content)

        print(f"MJCF file created successfully: {output_path}")

    except ImportError:
        print("ERROR: MuJoCo Python bindings not found.")
        print("Install with: pip install mujoco")
        print("\nCreating a template MJCF file instead...")

        mjcf_content = generate_mjcf_template()
        with open(output_path, 'w') as f:
            f.write(mjcf_content)

        print(f"Template MJCF file created: {output_path}")
        print("Please edit this file to match your robot's URDF.")


def generate_mjcf_wrapper(urdf_path: str) -> str:
    """Generate MJCF wrapper that includes URDF."""
    return f"""<mujoco model="humanoid">
  <compiler angle="radian" meshdir="../meshes" autolimits="true"/>

  <option timestep="0.002" iterations="50" solver="Newton" gravity="0 0 -9.81">
    <flag warmstart="enable"/>
  </option>

  <visual>
    <headlight ambient="0.5 0.5 0.5" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global offwidth="1920" offheight="1080"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>
    <texture name="texplane" type="2d" builtin="checker" rgb1="0.2 0.2 0.2" rgb2="0.3 0.3 0.3" width="512" height="512" mark="cross" markrgb="0.8 0.8 0.8"/>
    <material name="matplane" reflectance="0.3" texture="texplane" texrepeat="1 1" texuniform="true"/>
  </asset>

  <!-- Include URDF (MuJoCo 2.1+ supports URDF directly) -->
  <include file="{urdf_path}"/>

  <!-- Ground plane -->
  <worldbody>
    <light directional="true" diffuse="0.8 0.8 0.8" specular="0.2 0.2 0.2" pos="0 0 5" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="10 10 0.1" material="matplane" condim="3" friction="1 0.005 0.0001"/>
  </worldbody>

  <!-- Actuators (will be auto-generated from URDF transmission tags) -->
  <actuator>
    <!-- Position servos for all joints -->
    <!-- These will be populated based on your URDF joints -->
  </actuator>

  <!-- Sensors -->
  <sensor>
    <!-- IMU sensor -->
    <accelerometer name="imu_accel" site="imu_site"/>
    <gyro name="imu_gyro" site="imu_site"/>
    <magnetometer name="imu_mag" site="imu_site"/>

    <!-- Force-torque sensors for feet -->
    <force name="left_foot_force" site="left_foot_site"/>
    <torque name="left_foot_torque" site="left_foot_site"/>
    <force name="right_foot_force" site="right_foot_site"/>
    <torque name="right_foot_torque" site="right_foot_site"/>

    <!-- Joint position and velocity sensors -->
    <jointpos name="joint_pos"/>
    <jointvel name="joint_vel"/>

    <!-- Center of mass -->
    <framepos name="torso_pos" objtype="body" objname="torso"/>
    <framequat name="torso_quat" objtype="body" objname="torso"/>
    <framelinvel name="torso_linvel" objtype="body" objname="torso"/>
    <frameangvel name="torso_angvel" objtype="body" objname="torso"/>
  </sensor>
</mujoco>
"""


def generate_mjcf_template() -> str:
    """Generate a complete MJCF template for humanoid robot."""
    return """<mujoco model="humanoid">
  <compiler angle="radian" meshdir="meshes" autolimits="true"/>

  <option timestep="0.002" iterations="50" solver="Newton" gravity="0 0 -9.81">
    <flag warmstart="enable" constraint="enable"/>
  </option>

  <size njmax="1000" nconmax="400" nstack="1000000"/>

  <visual>
    <headlight ambient="0.5 0.5 0.5" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global offwidth="1920" offheight="1080"/>
    <quality shadowsize="4096"/>
    <map force="0.1" zfar="30"/>
  </visual>

  <statistic center="0 0 0.7" extent="1.5"/>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>
    <texture name="texplane" type="2d" builtin="checker" rgb1="0.2 0.2 0.2" rgb2="0.3 0.3 0.3" width="512" height="512" mark="cross" markrgb="0.8 0.8 0.8"/>
    <material name="matplane" reflectance="0.3" texture="texplane" texrepeat="1 1" texuniform="true"/>
    <material name="robot" rgba="0.8 0.8 0.9 1"/>
  </asset>

  <default>
    <joint limited="true" damping="0.5" armature="0.01"/>
    <geom contype="1" conaffinity="1" condim="3" friction="0.9 0.1 0.1" solimp="0.9 0.95 0.001" solref="0.002 1"/>
    <motor ctrllimited="true" ctrlrange="-1 1"/>
    <equality solref="0.002 1"/>
  </default>

  <worldbody>
    <light directional="true" diffuse="0.8 0.8 0.8" specular="0.2 0.2 0.2" pos="0 0 5" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="10 10 0.1" material="matplane"/>

    <!-- Humanoid Robot -->
    <body name="torso" pos="0 0 1.0">
      <camera name="track" mode="trackcom" pos="0 -3 1" xyaxes="1 0 0 0 0 1"/>
      <site name="imu_site" pos="0 0 0" size="0.01"/>
      <freejoint name="root"/>

      <geom name="torso_geom" type="box" size="0.15 0.1 0.25" rgba="0.8 0.8 0.9 1"/>
      <inertial pos="0 0 0" mass="10" diaginertia="0.1 0.1 0.05"/>

      <!-- Head -->
      <body name="head" pos="0 0 0.3">
        <joint name="neck_yaw" type="hinge" axis="0 0 1" range="-1.57 1.57"/>
        <geom name="head_geom" type="sphere" size="0.1" rgba="0.8 0.8 0.9 1"/>
        <inertial pos="0 0 0" mass="2" diaginertia="0.01 0.01 0.01"/>
      </body>

      <!-- Left Arm -->
      <body name="left_upper_arm" pos="0 0.2 0.15">
        <joint name="left_shoulder_pitch" type="hinge" axis="1 0 0" range="-3.14 3.14"/>
        <joint name="left_shoulder_roll" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
        <geom name="left_upper_arm_geom" type="capsule" fromto="0 0 0 0 0 -0.3" size="0.04"/>
        <inertial pos="0 0 -0.15" mass="1.5" diaginertia="0.01 0.01 0.002"/>

        <body name="left_lower_arm" pos="0 0 -0.3">
          <joint name="left_elbow_pitch" type="hinge" axis="1 0 0" range="0 2.5"/>
          <geom name="left_lower_arm_geom" type="capsule" fromto="0 0 0 0 0 -0.25" size="0.035"/>
          <inertial pos="0 0 -0.125" mass="1" diaginertia="0.005 0.005 0.001"/>

          <body name="left_hand" pos="0 0 -0.25">
            <joint name="left_wrist_roll" type="hinge" axis="0 0 1" range="-1.57 1.57"/>
            <geom name="left_hand_geom" type="box" size="0.04 0.06 0.02"/>
            <inertial pos="0 0 0" mass="0.3" diaginertia="0.001 0.001 0.001"/>
          </body>
        </body>
      </body>

      <!-- Right Arm (mirrored) -->
      <body name="right_upper_arm" pos="0 -0.2 0.15">
        <joint name="right_shoulder_pitch" type="hinge" axis="1 0 0" range="-3.14 3.14"/>
        <joint name="right_shoulder_roll" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
        <geom name="right_upper_arm_geom" type="capsule" fromto="0 0 0 0 0 -0.3" size="0.04"/>
        <inertial pos="0 0 -0.15" mass="1.5" diaginertia="0.01 0.01 0.002"/>

        <body name="right_lower_arm" pos="0 0 -0.3">
          <joint name="right_elbow_pitch" type="hinge" axis="1 0 0" range="0 2.5"/>
          <geom name="right_lower_arm_geom" type="capsule" fromto="0 0 0 0 0 -0.25" size="0.035"/>
          <inertial pos="0 0 -0.125" mass="1" diaginertia="0.005 0.005 0.001"/>

          <body name="right_hand" pos="0 0 -0.25">
            <joint name="right_wrist_roll" type="hinge" axis="0 0 1" range="-1.57 1.57"/>
            <geom name="right_hand_geom" type="box" size="0.04 0.06 0.02"/>
            <inertial pos="0 0 0" mass="0.3" diaginertia="0.001 0.001 0.001"/>
          </body>
        </body>
      </body>

      <!-- Left Leg -->
      <body name="left_upper_leg" pos="0 0.1 -0.25">
        <joint name="left_hip_pitch" type="hinge" axis="1 0 0" range="-1.57 1.57"/>
        <joint name="left_hip_roll" type="hinge" axis="0 1 0" range="-0.5 0.5"/>
        <joint name="left_hip_yaw" type="hinge" axis="0 0 1" range="-0.5 0.5"/>
        <geom name="left_upper_leg_geom" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.06"/>
        <inertial pos="0 0 -0.2" mass="5" diaginertia="0.05 0.05 0.01"/>

        <body name="left_lower_leg" pos="0 0 -0.4">
          <joint name="left_knee_pitch" type="hinge" axis="1 0 0" range="-2.5 0"/>
          <geom name="left_lower_leg_geom" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.05"/>
          <inertial pos="0 0 -0.2" mass="3" diaginertia="0.03 0.03 0.005"/>

          <body name="left_foot" pos="0 0 -0.4">
            <joint name="left_ankle_pitch" type="hinge" axis="1 0 0" range="-0.7 0.7"/>
            <joint name="left_ankle_roll" type="hinge" axis="0 1 0" range="-0.3 0.3"/>
            <site name="left_foot_site" pos="0 0 -0.05" size="0.01"/>
            <geom name="left_foot_geom" type="box" size="0.12 0.06 0.02" pos="0.03 0 -0.02"/>
            <inertial pos="0 0 -0.02" mass="1" diaginertia="0.005 0.005 0.005"/>
          </body>
        </body>
      </body>

      <!-- Right Leg (mirrored) -->
      <body name="right_upper_leg" pos="0 -0.1 -0.25">
        <joint name="right_hip_pitch" type="hinge" axis="1 0 0" range="-1.57 1.57"/>
        <joint name="right_hip_roll" type="hinge" axis="0 1 0" range="-0.5 0.5"/>
        <joint name="right_hip_yaw" type="hinge" axis="0 0 1" range="-0.5 0.5"/>
        <geom name="right_upper_leg_geom" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.06"/>
        <inertial pos="0 0 -0.2" mass="5" diaginertia="0.05 0.05 0.01"/>

        <body name="right_lower_leg" pos="0 0 -0.4">
          <joint name="right_knee_pitch" type="hinge" axis="1 0 0" range="-2.5 0"/>
          <geom name="right_lower_leg_geom" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.05"/>
          <inertial pos="0 0 -0.2" mass="3" diaginertia="0.03 0.03 0.005"/>

          <body name="right_foot" pos="0 0 -0.4">
            <joint name="right_ankle_pitch" type="hinge" axis="1 0 0" range="-0.7 0.7"/>
            <joint name="right_ankle_roll" type="hinge" axis="0 1 0" range="-0.3 0.3"/>
            <site name="right_foot_site" pos="0 0 -0.05" size="0.01"/>
            <geom name="right_foot_geom" type="box" size="0.12 0.06 0.02" pos="0.03 0 -0.02"/>
            <inertial pos="0 0 -0.02" mass="1" diaginertia="0.005 0.005 0.005"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- Head -->
    <position name="neck_yaw_motor" joint="neck_yaw" kp="100"/>

    <!-- Left Arm -->
    <position name="left_shoulder_pitch_motor" joint="left_shoulder_pitch" kp="200"/>
    <position name="left_shoulder_roll_motor" joint="left_shoulder_roll" kp="200"/>
    <position name="left_elbow_pitch_motor" joint="left_elbow_pitch" kp="100"/>
    <position name="left_wrist_roll_motor" joint="left_wrist_roll" kp="50"/>

    <!-- Right Arm -->
    <position name="right_shoulder_pitch_motor" joint="right_shoulder_pitch" kp="200"/>
    <position name="right_shoulder_roll_motor" joint="right_shoulder_roll" kp="200"/>
    <position name="right_elbow_pitch_motor" joint="right_elbow_pitch" kp="100"/>
    <position name="right_wrist_roll_motor" joint="right_wrist_roll" kp="50"/>

    <!-- Left Leg -->
    <position name="left_hip_pitch_motor" joint="left_hip_pitch" kp="500"/>
    <position name="left_hip_roll_motor" joint="left_hip_roll" kp="500"/>
    <position name="left_hip_yaw_motor" joint="left_hip_yaw" kp="500"/>
    <position name="left_knee_pitch_motor" joint="left_knee_pitch" kp="500"/>
    <position name="left_ankle_pitch_motor" joint="left_ankle_pitch" kp="300"/>
    <position name="left_ankle_roll_motor" joint="left_ankle_roll" kp="300"/>

    <!-- Right Leg -->
    <position name="right_hip_pitch_motor" joint="right_hip_pitch" kp="500"/>
    <position name="right_hip_roll_motor" joint="right_hip_roll" kp="500"/>
    <position name="right_hip_yaw_motor" joint="right_hip_yaw" kp="500"/>
    <position name="right_knee_pitch_motor" joint="right_knee_pitch" kp="500"/>
    <position name="right_ankle_pitch_motor" joint="right_ankle_pitch" kp="300"/>
    <position name="right_ankle_roll_motor" joint="right_ankle_roll" kp="300"/>
  </actuator>

  <sensor>
    <!-- IMU sensor -->
    <accelerometer name="imu_accel" site="imu_site"/>
    <gyro name="imu_gyro" site="imu_site"/>

    <!-- Foot force sensors -->
    <force name="left_foot_force" site="left_foot_site"/>
    <torque name="left_foot_torque" site="left_foot_site"/>
    <force name="right_foot_force" site="right_foot_site"/>
    <torque name="right_foot_torque" site="right_foot_site"/>

    <!-- Torso pose -->
    <framepos name="torso_pos" objtype="body" objname="torso"/>
    <framequat name="torso_quat" objtype="body" objname="torso"/>
    <framelinvel name="torso_linvel" objtype="body" objname="torso"/>
    <frameangvel name="torso_angvel" objtype="body" objname="torso"/>
  </sensor>

  <keyframe>
    <key name="standing" qpos="0 0 1.0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"/>
  </keyframe>
</mujoco>
"""


def main():
    parser = argparse.ArgumentParser(description='Convert URDF to MuJoCo MJCF')
    parser.add_argument('--urdf', type=str, help='Path to URDF file')
    parser.add_argument('--output', type=str, help='Output MJCF file path')
    parser.add_argument('--template', action='store_true', help='Generate template MJCF instead of converting')

    args = parser.parse_args()

    if args.template or not args.urdf:
        # Generate template
        output_path = args.output or 'humanoid.xml'
        mjcf_content = generate_mjcf_template()
        with open(output_path, 'w') as f:
            f.write(mjcf_content)
        print(f"Template MJCF created: {output_path}")
    else:
        # Convert URDF
        convert_urdf_to_mjcf(args.urdf, args.output or 'humanoid.xml')


if __name__ == '__main__':
    main()
