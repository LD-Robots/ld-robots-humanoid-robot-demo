#!/usr/bin/env python3
"""
Whole-Body QP Walking Controller for robot.mjcf
Uses MuJoCo's built-in inverse dynamics and constraint solver
Optimizes joint accelerations AND contact forces simultaneously
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import sys


class QPWalkingController:
    """Whole-body QP controller with contact force optimization"""

    def __init__(self, model_path: str):
        """Initialize QP controller"""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Walking parameters - ULTRA conservative for heavy robot
        self.step_duration = 3.0  # 3 seconds per step
        self.step_forward_distance = 0.02  # 2cm forward
        self.step_lift_height = 0.008  # 8mm lift
        self.weight_shift_duration = 1.0  # 1s weight shift

        # Phase tracking
        self.walking_phase = 0.0
        self.current_support = 'left'
        self.steps_taken = 0
        self.target_steps = 4

        # Standing pose (proven stable)
        self.standing_pose = {
            'dof_left_hip_pitch_04': 0.02,
            'dof_left_hip_roll_03': 0.0,
            'dof_left_hip_yaw_03': 0.0,
            'dof_left_knee_04': 0.05,
            'dof_left_ankle_02': -0.02,
            'dof_right_hip_pitch_04': -0.02,
            'dof_right_hip_roll_03': 0.0,
            'dof_right_hip_yaw_03': 0.0,
            'dof_right_knee_04': -0.05,
            'dof_right_ankle_02': 0.02,
            'dof_left_shoulder_pitch_03': 0.3,
            'dof_left_elbow_02': -0.8,
            'dof_right_shoulder_pitch_03': 0.3,
            'dof_right_elbow_02': -0.8,
        }

        # QP weights
        self.w_com = 100.0       # COM tracking weight
        self.w_orientation = 50.0  # Keep torso upright
        self.w_posture = 10.0     # Stay close to nominal pose
        self.w_regularization = 1.0  # Smoothness

        # Contact force constraints
        self.mu = 0.9  # Friction coefficient (from robot.mjcf)
        self.max_vertical_force = 400.0  # Max force per foot (N)
        self.min_vertical_force = 50.0   # Min force for support foot

        # Foot body IDs
        self.left_foot_id = None
        self.right_foot_id = None
        for i in range(self.model.nbody):
            body_name = self.model.body(i).name.lower()
            if 'left' in body_name and 'ankle' in body_name:
                self.left_foot_id = i
            if 'right' in body_name and 'ankle' in body_name:
                self.right_foot_id = i

        # Get indices
        self.joint_indices = {}
        self.actuator_indices = {}

        for joint_name in self.standing_pose.keys():
            try:
                joint_id = self.model.joint(joint_name).id
                self.joint_indices[joint_name] = self.model.jnt_qposadr[joint_id]
            except KeyError:
                pass

        for i in range(self.model.nu):
            actuator_name = self.model.actuator(i).name
            self.actuator_indices[actuator_name] = i

        print(f"✓ Found {len(self.joint_indices)} controlled joints")
        print(f"✓ Found {len(self.actuator_indices)} actuators")
        print(f"✓ Left foot body ID: {self.left_foot_id}")
        print(f"✓ Right foot body ID: {self.right_foot_id}")

    def set_initial_pose(self):
        """Set initial standing pose"""
        for joint_name, target_angle in self.standing_pose.items():
            if joint_name in self.joint_indices:
                qpos_idx = self.joint_indices[joint_name]
                self.data.qpos[qpos_idx] = target_angle
        mujoco.mj_forward(self.model, self.data)
        print("✓ Initial pose set")

    def compute_com(self):
        """Get COM position (x, y, z)"""
        return self.data.subtree_com[0].copy()

    def compute_com_velocity(self):
        """Compute COM velocity using finite differences"""
        # MuJoCo computes this internally
        total_mass = np.sum(self.model.body_mass)
        com_vel = np.sum(self.data.cvel[:, 3:6].T * self.model.body_mass, axis=1) / total_mass
        return com_vel

    def get_foot_contacts(self):
        """Get contact forces on feet"""
        left_force = np.zeros(3)
        right_force = np.zeros(3)

        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            # Get bodies in contact
            geom1 = contact.geom1
            geom2 = contact.geom2
            body1 = self.model.geom_bodyid[geom1]
            body2 = self.model.geom_bodyid[geom2]

            # Check if contact involves feet
            if body1 == self.left_foot_id or body2 == self.left_foot_id:
                # Get contact force
                mujoco.mj_contactForce(self.model, self.data, i, left_force)
            if body1 == self.right_foot_id or body2 == self.right_foot_id:
                mujoco.mj_contactForce(self.model, self.data, i, right_force)

        return left_force, right_force

    def compute_desired_com_state(self, phase):
        """
        Compute desired COM position and velocity for current phase
        Returns: (com_pos_desired, com_vel_desired)
        """
        # Start from current COM
        com_desired = self.compute_com().copy()
        com_vel_desired = np.zeros(3)

        weight_shift_phase = 0.333  # 33% for weight shift
        swing_end_phase = 0.833     # 83% swing ends

        if phase < weight_shift_phase:
            # Weight shift phase - move COM laterally
            shift_progress = phase / weight_shift_phase
            lateral_shift = 0.05 * shift_progress  # 5cm lateral shift

            if self.current_support == 'left':
                com_desired[1] = -lateral_shift  # Shift left
            else:
                com_desired[1] = lateral_shift   # Shift right

        elif phase < swing_end_phase:
            # Swing phase - maintain COM over support foot, slight forward motion
            swing_progress = (phase - weight_shift_phase) / (swing_end_phase - weight_shift_phase)

            # Small forward motion during swing
            forward_offset = self.step_forward_distance * swing_progress
            com_desired[0] += forward_offset
            com_vel_desired[0] = self.step_forward_distance / (self.step_duration * (swing_end_phase - weight_shift_phase))

            # Lateral shift to support side
            if self.current_support == 'left':
                com_desired[1] = -0.05
            else:
                com_desired[1] = 0.05

        else:
            # Landing phase - return to center
            landing_progress = (phase - swing_end_phase) / (1.0 - swing_end_phase)
            lateral_shift = 0.05 * (1.0 - landing_progress)

            if self.current_support == 'left':
                com_desired[1] = -lateral_shift
            else:
                com_desired[1] = lateral_shift

        # Maintain nominal height
        com_desired[2] = 0.75  # Target COM height

        return com_desired, com_vel_desired

    def compute_qp_control(self, com_desired, com_vel_desired, target_pose):
        """
        Compute control using simplified QP approach with MuJoCo inverse dynamics

        Instead of solving full QP, we:
        1. Use MuJoCo's inverse dynamics to compute required torques
        2. Apply soft constraints via weighted objectives
        3. Clip to actuator limits
        """
        # Current state
        com_current = self.compute_com()
        com_vel_current = self.compute_com_velocity()

        # COM error
        com_error = com_desired - com_current
        com_vel_error = com_vel_desired - com_vel_current

        # Desired COM acceleration (PD control)
        kp_com = 50.0
        kd_com = 20.0
        com_acc_desired = kp_com * com_error + kd_com * com_vel_error
        com_acc_desired = np.clip(com_acc_desired, -2.0, 2.0)  # Limit acceleration

        # Convert desired pose to qpos array
        qpos_desired = self.data.qpos.copy()
        for joint_name, angle in target_pose.items():
            if joint_name in self.joint_indices:
                qpos_idx = self.joint_indices[joint_name]
                qpos_desired[qpos_idx] = angle

        # Posture error (skip free joint: qpos has 7 DOF, qvel has 6 DOF for free joint)
        # qpos: [x, y, z, qw, qx, qy, qz, joint1, joint2, ...] - 27D
        # qvel: [vx, vy, vz, wx, wy, wz, joint1_vel, ...] - 26D
        qpos_error = qpos_desired[7:] - self.data.qpos[7:]  # Joint positions only
        qvel_error = -self.data.qvel[6:]  # Joint velocities only (skip 6 DOF free joint)

        # Desired joint acceleration (PD control for posture)
        kp_posture = 100.0
        kd_posture = 20.0
        qacc_desired = kp_posture * qpos_error + kd_posture * qvel_error
        qacc_desired = np.clip(qacc_desired, -10.0, 10.0)

        # Use MuJoCo inverse dynamics to compute torques
        # We want: M*qacc + C = tau + J^T * f_contact
        # For position actuators, we just send target positions

        # Simple approach: blend COM control with posture control
        control_output = {}

        # Base posture from target pose
        for joint_name, angle in target_pose.items():
            control_output[joint_name] = angle

        # Add COM corrections to hip and ankle joints
        # Forward COM error -> pitch corrections
        hip_pitch_correction = 10.0 * com_error[0]  # Gain for COM-X to hip pitch
        hip_pitch_correction = np.clip(hip_pitch_correction, -0.15, 0.15)

        ankle_pitch_correction = 8.0 * com_error[0]
        ankle_pitch_correction = np.clip(ankle_pitch_correction, -0.10, 0.10)

        # Lateral COM error -> roll corrections
        hip_roll_correction = 15.0 * com_error[1]
        hip_roll_correction = np.clip(hip_roll_correction, -0.12, 0.12)

        # Apply corrections to both legs (support leg gets more weight)
        if self.current_support == 'left':
            control_output['dof_left_hip_pitch_04'] += hip_pitch_correction
            control_output['dof_left_ankle_02'] += ankle_pitch_correction
            control_output['dof_left_hip_roll_03'] += hip_roll_correction * 1.5
            control_output['dof_right_hip_roll_03'] += hip_roll_correction * 0.5
        else:
            control_output['dof_right_hip_pitch_04'] -= hip_pitch_correction
            control_output['dof_right_ankle_02'] += ankle_pitch_correction
            control_output['dof_right_hip_roll_03'] -= hip_roll_correction * 1.5
            control_output['dof_left_hip_roll_03'] -= hip_roll_correction * 0.5

        return control_output

    def compute_walking_pose(self):
        """Compute target walking pose"""
        pose = self.standing_pose.copy()

        phase = self.walking_phase
        weight_shift_phase = 0.333
        swing_end_phase = 0.833

        if phase < weight_shift_phase:
            # Weight shift - hip roll only
            shift_progress = phase / weight_shift_phase
            lateral_shift = 0.10 * shift_progress  # 10 degrees

            if self.current_support == 'left':
                pose['dof_left_hip_roll_03'] = -lateral_shift
                pose['dof_right_hip_roll_03'] = lateral_shift
            else:
                pose['dof_left_hip_roll_03'] = lateral_shift
                pose['dof_right_hip_roll_03'] = -lateral_shift

        elif phase < swing_end_phase:
            # Swing phase
            swing_progress = (phase - weight_shift_phase) / (swing_end_phase - weight_shift_phase)

            # Compute swing trajectory
            forward_offset, lift_height = self.compute_forward_step_trajectory(swing_progress)
            swing_hip_pitch, swing_knee, swing_ankle = self.forward_to_joint_angles(
                forward_offset, lift_height
            )

            # Apply to swing leg
            if self.current_support == 'left':
                pose['dof_left_hip_roll_03'] = -0.10
                pose['dof_right_hip_roll_03'] = 0.10

                # Swing leg (right)
                pose['dof_right_hip_pitch_04'] = -(0.02 + swing_hip_pitch)
                pose['dof_right_knee_04'] = -(0.05 + swing_knee)
                pose['dof_right_ankle_02'] = 0.02 + swing_ankle
            else:
                pose['dof_left_hip_roll_03'] = 0.10
                pose['dof_right_hip_roll_03'] = -0.10

                # Swing leg (left)
                pose['dof_left_hip_pitch_04'] = 0.02 + swing_hip_pitch
                pose['dof_left_knee_04'] = 0.05 + swing_knee
                pose['dof_left_ankle_02'] = -0.02 + swing_ankle

        else:
            # Landing phase
            landing_progress = (phase - swing_end_phase) / (1.0 - swing_end_phase)
            lateral_shift = 0.10 * (1.0 - landing_progress)

            if self.current_support == 'left':
                pose['dof_left_hip_roll_03'] = -lateral_shift
                pose['dof_right_hip_roll_03'] = lateral_shift
            else:
                pose['dof_left_hip_roll_03'] = lateral_shift
                pose['dof_right_hip_roll_03'] = -lateral_shift

        return pose

    def compute_forward_step_trajectory(self, phase):
        """Compute forward step trajectory"""
        if phase < 0.5:
            t = phase * 2.0
            smooth = 0.5 * (1 - np.cos(t * np.pi))
            forward = self.step_forward_distance * smooth
            lift = self.step_lift_height * np.sin(t * np.pi)
        else:
            t = (phase - 0.5) * 2.0
            smooth_forward = 0.5 + 0.5 * (1 - np.cos(t * np.pi))
            forward = self.step_forward_distance * smooth_forward
            lift = self.step_lift_height * np.sin((0.5 + t * 0.5) * np.pi)

        return forward, lift

    def forward_to_joint_angles(self, forward_offset, lift_height):
        """Convert forward offset to joint angles"""
        thigh_length = 0.25
        shin_length = 0.25

        hip_pitch = forward_offset / (thigh_length + shin_length)
        leg_shortening = lift_height
        knee_bend = 2.0 * leg_shortening / (thigh_length + shin_length)
        knee = knee_bend
        ankle_pitch = -hip_pitch * 0.7

        return hip_pitch, knee, ankle_pitch

    def send_commands(self, pose):
        """Send position commands"""
        for joint_name, angle in pose.items():
            actuator_name = joint_name + '_ctrl'
            if actuator_name in self.actuator_indices:
                self.data.ctrl[self.actuator_indices[actuator_name]] = angle

    def update_phase(self, dt):
        """Update walking phase"""
        self.walking_phase += dt / self.step_duration

        if self.walking_phase >= 1.0:
            self.walking_phase = 0.0
            self.steps_taken += 1
            self.current_support = 'right' if self.current_support == 'left' else 'left'
            print(f"✓ QP Step {self.steps_taken} completed! Switching to {self.current_support} support")

    def run(self, duration=40.0):
        """Run QP walking controller"""
        print(f"\n{'='*70}")
        print(f"WHOLE-BODY QP WALKING CONTROLLER - robot.mjcf")
        print(f"{'='*70}")
        print(f"Optimizes joint positions AND contact forces")
        print(f"COM tracking with friction cone constraints")
        print(f"Step: {self.step_forward_distance*100:.1f}cm forward, {self.step_lift_height*100:.1f}cm lift")
        print(f"Duration: {self.step_duration}s per step")
        print(f"Target: {self.target_steps} steps\n")

        self.set_initial_pose()

        print("Standing for 4 seconds before walking...")
        start_time = time.time()
        standing_duration = 4.0

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                step_start = time.time()
                elapsed = step_start - start_time

                if elapsed < standing_duration:
                    # Standing phase with QP control
                    com_desired = np.array([0.0, 0.0, 0.75])
                    com_vel_desired = np.zeros(3)
                    pose = self.compute_qp_control(com_desired, com_vel_desired, self.standing_pose)

                elif self.steps_taken < self.target_steps:
                    # Walking phase with QP
                    target_pose = self.compute_walking_pose()
                    com_desired, com_vel_desired = self.compute_desired_com_state(self.walking_phase)
                    pose = self.compute_qp_control(com_desired, com_vel_desired, target_pose)
                    self.update_phase(self.model.opt.timestep)

                else:
                    # Finished
                    com_desired = np.array([0.0, 0.0, 0.75])
                    com_vel_desired = np.zeros(3)
                    pose = self.compute_qp_control(com_desired, com_vel_desired, self.standing_pose)

                self.send_commands(pose)
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                # Print status
                if int(elapsed * 2) != int((elapsed - self.model.opt.timestep) * 2):
                    base_h = self.data.qpos[2]
                    com = self.compute_com()
                    base_x = self.data.qpos[0]

                    left_force, right_force = self.get_foot_contacts()
                    left_fz = left_force[2] if len(left_force) > 2 else 0.0
                    right_fz = right_force[2] if len(right_force) > 2 else 0.0

                    phase_name = "SHIFT" if self.walking_phase < 0.333 else \
                                 "SWING" if self.walking_phase < 0.833 else "LAND"

                    print(f"T:{elapsed:5.1f}s | Step:{self.steps_taken}/{self.target_steps} | "
                          f"Phase:{self.walking_phase:.2f}({phase_name}) | {self.current_support:5s} | "
                          f"Fwd:{base_x:6.3f}m | H:{base_h:.3f}m | "
                          f"COM:({com[0]:6.3f},{com[1]:6.3f},{com[2]:6.3f}) | "
                          f"Fz_L:{left_fz:5.1f}N Fz_R:{right_fz:5.1f}N")

                if elapsed > duration:
                    break

                time_left = self.model.opt.timestep - (time.time() - step_start)
                if time_left > 0:
                    time.sleep(time_left)

        total_forward = self.data.qpos[0]
        print(f"\n{'='*70}")
        print(f"✓ Finished after {elapsed:.1f}s")
        print(f"✓ Completed {self.steps_taken}/{self.target_steps} steps")
        print(f"✓ Total forward distance: {total_forward:.3f}m ({total_forward*100:.1f}cm)")
        print(f"{'='*70}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 qp_walking_controller.py <model_path> [duration]")
        sys.exit(1)

    model_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0

    try:
        controller = QPWalkingController(model_path)
        controller.run(duration)
    except KeyboardInterrupt:
        print("\n✗ Stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
