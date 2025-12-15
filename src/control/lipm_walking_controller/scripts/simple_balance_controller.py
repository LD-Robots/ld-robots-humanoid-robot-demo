#!/usr/bin/env python3
"""
Simple balance controller to keep the humanoid robot standing.

Uses PD control on joints to maintain a stable standing pose.
"""

import numpy as np
import mujoco
import mujoco.viewer
import sys


class SimpleBalanceController:
    """Simple PD controller for standing balance."""

    def __init__(self, model_path: str):
        """Initialize MuJoCo simulation and controller."""
        print(f"Loading MuJoCo model: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        print(f"✓ Model loaded: {self.model.nbody} bodies, {self.model.njnt} joints, {self.model.nu} actuators")

        # Control parameters
        self.dt = self.model.opt.timestep  # Use model timestep
        self.step_count = 0  # Initialize step counter

        # PD gains for different joint types (scaled for motor torque limits)
        # Hip/Knee joints (RS04 motors: ±120 Nm) - high torque, stiffer control
        self.kp_hip_knee = 500.0  # Proportional gain (Nm/rad)
        self.kd_hip_knee = 50.0   # Derivative gain (Nm·s/rad)

        # Hip roll/yaw, Shoulder (RS03 motors: ±60 Nm) - medium torque
        self.kp_medium = 300.0
        self.kd_medium = 30.0

        # Ankle, Elbow joints (RS02 motors: ±17 Nm) - lower torque
        self.kp_ankle = 100.0
        self.kd_ankle = 10.0

        # Wrist joints (RS00: ±14 Nm) - very soft
        self.kp_wrist = 50.0
        self.kd_wrist = 5.0

        # Target standing pose (all angles in radians)
        # Conservative pose close to neutral for stability
        self.target_pose = {
            # Left leg
            'dof_left_hip_pitch_04': 0.0,     # Neutral
            'dof_left_hip_roll_03': 0.0,      # Neutral
            'dof_left_hip_yaw_03': 0.0,       # Neutral
            'dof_left_knee_04': 0.0,          # Straight
            'dof_left_ankle_02': 0.0,         # Neutral

            # Right leg
            'dof_right_hip_pitch_04': 0.0,    # Neutral
            'dof_right_hip_roll_03': 0.0,     # Neutral
            'dof_right_hip_yaw_03': 0.0,      # Neutral
            'dof_right_knee_04': 0.0,         # Straight
            'dof_right_ankle_02': 0.0,        # Neutral

            # Arms - relaxed at sides
            'dof_left_shoulder_pitch_03': 0.5,    # Slightly forward (range: -60° to 200°)
            'dof_left_shoulder_roll_03': 0.3,     # Slightly out (range: -25° to 95°)
            'dof_left_shoulder_yaw_02': 0.0,
            'dof_left_elbow_02': -0.3,            # Bent elbow (range: -142° to 0°)
            'dof_left_wrist_00': 0.0,

            'dof_right_shoulder_pitch_03': -0.5,  # Slightly back (range: -200° to 60°)
            'dof_right_shoulder_roll_03': -0.3,   # Slightly out (range: -95° to 25°)
            'dof_right_shoulder_yaw_02': 0.0,
            'dof_right_elbow_02': 0.3,            # Bent elbow (range: 0° to 142°)
            'dof_right_wrist_00': 0.0,
        }

        # Build joint index mapping
        self.joint_indices = {}
        self.actuator_indices = {}

        for i in range(self.model.njnt):
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if joint_name != 'floating_base':  # Skip freejoint
                self.joint_indices[joint_name] = i

        for i in range(self.model.nu):
            actuator_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            self.actuator_indices[actuator_name] = i

        print(f"✓ Found {len(self.joint_indices)} controlled joints")
        print(f"✓ Found {len(self.actuator_indices)} actuators")

        # Set initial pose
        self._set_initial_pose()

    def _set_initial_pose(self):
        """Set robot to initial standing pose."""
        # Reset all qpos
        self.data.qpos[:] = 0.0

        # Set floating base position (x, y, z, qw, qx, qy, qz)
        self.data.qpos[0] = 0.0   # X position
        self.data.qpos[1] = 0.0   # Y position
        self.data.qpos[2] = 0.85  # Z position (standing height)
        self.data.qpos[3] = 1.0   # qw (quaternion w)
        self.data.qpos[4:7] = 0.0 # qx, qy, qz

        # Set joint angles to target pose
        for joint_name, target_angle in self.target_pose.items():
            if joint_name in self.joint_indices:
                joint_idx = self.joint_indices[joint_name]
                qpos_idx = self.model.jnt_qposadr[joint_idx]
                self.data.qpos[qpos_idx] = target_angle

        # Zero velocities
        self.data.qvel[:] = 0.0

        # Forward kinematics to update positions
        mujoco.mj_forward(self.model, self.data)

        print("✓ Initial pose set")

    def compute_pd_control(self):
        """Compute PD control torques for all joints."""
        # Zero all control inputs
        self.data.ctrl[:] = 0.0

        # Compute control for each joint
        for joint_name, target_angle in self.target_pose.items():
            if joint_name not in self.joint_indices:
                continue

            joint_idx = self.joint_indices[joint_name]
            qpos_idx = self.model.jnt_qposadr[joint_idx]
            qvel_idx = self.model.jnt_dofadr[joint_idx]

            # Current state
            current_angle = self.data.qpos[qpos_idx]
            current_velocity = self.data.qvel[qvel_idx]

            # Select gains based on joint type and motor class
            if 'hip_pitch' in joint_name or 'knee' in joint_name:
                # RS04 motors - strongest
                kp = self.kp_hip_knee
                kd = self.kd_hip_knee
            elif 'hip_roll' in joint_name or 'hip_yaw' in joint_name or 'shoulder_pitch' in joint_name or 'shoulder_roll' in joint_name:
                # RS03 motors - medium
                kp = self.kp_medium
                kd = self.kd_medium
            elif 'ankle' in joint_name or 'elbow' in joint_name or 'shoulder_yaw' in joint_name:
                # RS02 motors - lower torque
                kp = self.kp_ankle
                kd = self.kd_ankle
            else:  # wrist - RS00
                kp = self.kp_wrist
                kd = self.kd_wrist

            # PD control law
            error = target_angle - current_angle
            torque = kp * error - kd * current_velocity

            # Find corresponding actuator (actuators have _ctrl suffix)
            actuator_name = joint_name + '_ctrl'

            if actuator_name in self.actuator_indices:
                actuator_idx = self.actuator_indices[actuator_name]
                self.data.ctrl[actuator_idx] = torque
            else:
                # Debug: print missing actuator mapping
                if self.step_count < 10:
                    print(f"Warning: No actuator found for joint {joint_name}")

    def run(self, duration: float = 10.0):
        """Run the simulation with the controller."""
        print(f"\nStarting simulation for {duration} seconds...")
        print("Controls:")
        print("  - Spacebar: Pause/Resume")
        print("  - Mouse drag: Rotate view")
        print("  - Scroll: Zoom")
        print("  - Ctrl+Q or ESC: Quit")
        print("\nPress Ctrl+C to stop\n")

        self.step_count = 0
        self.start_time = self.data.time

        def controller_callback(model, data):
            """Called at each simulation step."""
            # Compute and apply control
            self.compute_pd_control()
            self.step_count += 1

            # Print debug info every 100 steps
            if self.step_count % 500 == 0:
                base_z = data.qpos[2]  # Z position of floating base
                print(f"Time: {data.time:.2f}s | Base height: {base_z:.3f}m | Steps: {self.step_count}")

        # Launch viewer with controller
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # Set camera position for better view
            viewer.cam.distance = 3.0
            viewer.cam.azimuth = 45
            viewer.cam.elevation = -20

            # Simulation loop
            while viewer.is_running() and (self.data.time - self.start_time) < duration:
                # Compute control
                controller_callback(self.model, self.data)

                # Step simulation
                mujoco.mj_step(self.model, self.data)

                # Update viewer
                viewer.sync()

        print(f"\n✓ Simulation completed: {self.data.time:.2f}s, {self.step_count} steps")
        print(f"  Final base height: {self.data.qpos[2]:.3f}m")


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage: python3 simple_balance_controller.py <model_path> [duration]")
        print("\nExample:")
        print("  python3 simple_balance_controller.py models/robot.mjcf 20")
        sys.exit(1)

    model_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) >= 3 else 30.0

    print("\n" + "="*60)
    print("SIMPLE BALANCE CONTROLLER - STANDING DEMO")
    print("="*60)
    print(f"Model: {model_path}")
    print(f"Duration: {duration}s")
    print("="*60 + "\n")

    try:
        controller = SimpleBalanceController(model_path)
        controller.run(duration=duration)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
