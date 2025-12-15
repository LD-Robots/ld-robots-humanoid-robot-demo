#!/usr/bin/env python3
"""
ZMP Walking Controller - INCREMENTAL walking based on proven zmp_balance_controller
Starts with 120s proven standing, then adds TINY walking motions
"""

import numpy as np
import mujoco
import mujoco.viewer
import sys
import time


class ZMPWalkingIncremental:
    """Walking controller based on proven ZMP standing controller."""

    def __init__(self, model_path: str):
        """Initialize MuJoCo simulation and controller."""
        print(f"Loading MuJoCo model: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        print(f"✓ Model loaded: {self.model.nbody} bodies, {self.model.njnt} joints, {self.model.nu} actuators")

        # Control parameters
        self.dt = self.model.opt.timestep

        # COM/ZMP control gains - EXACT from zmp_balance_controller
        self.hip_correction_gain = 3.0
        self.ankle_correction_gain = 6.0

        # Target standing pose - EXACT from zmp_balance_controller
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
            'dof_left_shoulder_roll_03': 0.2,
            'dof_left_shoulder_yaw_02': 0.0,
            'dof_left_elbow_02': -0.3,
            'dof_left_wrist_00': 0.0,
            'dof_right_shoulder_pitch_03': -0.3,
            'dof_right_shoulder_roll_03': -0.2,
            'dof_right_shoulder_yaw_02': 0.0,
            'dof_right_elbow_02': 0.3,
            'dof_right_wrist_00': 0.0,
        }

        # Walking state
        self.walking_phase = 0.0
        self.current_support = 'left'
        self.steps_taken = 0
        self.target_steps = 4

        # ULTRA-CONSERVATIVE walking parameters
        self.step_duration = 6.0  # 6 seconds per step (very slow!)
        self.step_forward = 0.01  # Only 1cm forward
        self.step_lift = 0.005    # Only 5mm lift
        self.weight_shift = 0.05  # Only 5 degrees lateral shift

        # Balance correction offsets
        self.hip_pitch_correction = 0.0
        self.ankle_pitch_correction = 0.0

        # Build joint/actuator mappings
        self.joint_indices = {}
        self.actuator_indices = {}

        for i in range(self.model.njnt):
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if joint_name != 'floating_base':
                self.joint_indices[joint_name] = i

        for i in range(self.model.nu):
            actuator_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            self.actuator_indices[actuator_name] = i

        print(f"✓ Found {len(self.joint_indices)} controlled joints")
        print(f"✓ Found {len(self.actuator_indices)} actuators")

        # Initialize robot pose - EXACT from zmp_balance_controller
        self._set_initial_pose()

        # Get foot body indices
        self.left_foot_body_id = None
        self.right_foot_body_id = None

        for i in range(self.model.nbody):
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if 'ankle' in body_name.lower() and 'left' in body_name.lower():
                self.left_foot_body_id = i
            elif 'ankle' in body_name.lower() and 'right' in body_name.lower():
                self.right_foot_body_id = i

    def _set_initial_pose(self):
        """Set robot to initial standing pose - EXACT from zmp_balance_controller."""
        self.data.qpos[:] = 0.0

        # Floating base (x, y, z, qw, qx, qy, qz) - CRITICAL: Set height to 0.85!
        self.data.qpos[0] = 0.0
        self.data.qpos[1] = 0.0
        self.data.qpos[2] = 0.85  # THIS IS THE KEY - from zmp_balance_controller!
        self.data.qpos[3] = 1.0   # qw
        self.data.qpos[4:7] = 0.0

        # Set target joint angles
        for joint_name, target_angle in self.standing_pose.items():
            if joint_name in self.joint_indices:
                joint_idx = self.joint_indices[joint_name]
                qpos_idx = self.model.jnt_qposadr[joint_idx]
                self.data.qpos[qpos_idx] = target_angle

        # Zero velocities
        self.data.qvel[:] = 0.0

        # Forward kinematics
        mujoco.mj_forward(self.model, self.data)

        print("✓ Initial pose set (height=0.85m)")

    def compute_com(self):
        """Compute center of mass position."""
        return self.data.subtree_com[0].copy()

    def compute_support_polygon_center(self):
        """Compute center of support polygon."""
        if self.left_foot_body_id is None or self.right_foot_body_id is None:
            return np.array([0.0, 0.0, 0.0])

        left_foot_pos = self.data.xpos[self.left_foot_body_id].copy()
        right_foot_pos = self.data.xpos[self.right_foot_body_id].copy()
        center = (left_foot_pos + right_foot_pos) / 2.0
        return center

    def compute_com_error(self):
        """Compute COM error relative to support polygon center."""
        com = self.compute_com()
        support_center = self.compute_support_polygon_center()
        error_x = com[0] - support_center[0]
        error_y = com[1] - support_center[1]
        return error_x, error_y

    def update_balance_corrections(self):
        """Update hip and ankle corrections - EXACT from zmp_balance_controller."""
        error_x, error_y = self.compute_com_error()

        # Hip strategy
        self.hip_pitch_correction = -self.hip_correction_gain * error_x
        self.hip_pitch_correction = np.clip(self.hip_pitch_correction, -0.4, 0.4)

        # Ankle strategy
        self.ankle_pitch_correction = -self.ankle_correction_gain * error_x
        self.ankle_pitch_correction = np.clip(self.ankle_pitch_correction, -0.3, 0.3)

    def compute_walking_pose(self):
        """Compute walking pose with TINY incremental motion."""
        pose = self.standing_pose.copy()

        phase = self.walking_phase
        weight_shift_end = 0.4  # 40% of step for weight shift
        swing_end = 0.9         # 90% for swing

        if phase < weight_shift_end:
            # Weight shift phase
            shift_progress = phase / weight_shift_end
            lateral_shift = self.weight_shift * shift_progress

            if self.current_support == 'left':
                pose['dof_left_hip_roll_03'] = -lateral_shift
                pose['dof_right_hip_roll_03'] = lateral_shift
            else:
                pose['dof_left_hip_roll_03'] = lateral_shift
                pose['dof_right_hip_roll_03'] = -lateral_shift

        elif phase < swing_end:
            # Swing phase - TINY forward step
            swing_progress = (phase - weight_shift_end) / (swing_end - weight_shift_end)

            # Smooth trajectory
            t = swing_progress
            smooth = 0.5 * (1 - np.cos(t * np.pi))
            forward = self.step_forward * smooth
            lift = self.step_lift * np.sin(t * np.pi)

            # Maintain weight shift
            if self.current_support == 'left':
                pose['dof_left_hip_roll_03'] = -self.weight_shift
                pose['dof_right_hip_roll_03'] = self.weight_shift

                # Right leg swings forward (TINY motion)
                pose['dof_right_hip_pitch_04'] = -0.02 - forward / 0.5  # forward offset
                pose['dof_right_knee_04'] = -0.05 - lift / 0.25       # lift
                pose['dof_right_ankle_02'] = 0.02 + forward / 0.5     # compensation
            else:
                pose['dof_left_hip_roll_03'] = self.weight_shift
                pose['dof_right_hip_roll_03'] = -self.weight_shift

                # Left leg swings forward (TINY motion)
                pose['dof_left_hip_pitch_04'] = 0.02 + forward / 0.5
                pose['dof_left_knee_04'] = 0.05 + lift / 0.25
                pose['dof_left_ankle_02'] = -0.02 - forward / 0.5

        else:
            # Landing phase - return to double support
            landing_progress = (phase - swing_end) / (1.0 - swing_end)
            lateral_shift = self.weight_shift * (1.0 - landing_progress)

            if self.current_support == 'left':
                pose['dof_left_hip_roll_03'] = -lateral_shift
                pose['dof_right_hip_roll_03'] = lateral_shift
            else:
                pose['dof_left_hip_roll_03'] = lateral_shift
                pose['dof_right_hip_roll_03'] = -lateral_shift

        return pose

    def compute_pd_control(self, walking_enabled=False):
        """Compute PD control - EXACT method from zmp_balance_controller."""
        self.update_balance_corrections()

        # Decide pose
        if walking_enabled and self.steps_taken < self.target_steps:
            base_pose = self.compute_walking_pose()
        else:
            base_pose = self.standing_pose

        # Apply PD control to each joint - EXACT from zmp_balance_controller
        for joint_name, base_target in base_pose.items():
            if joint_name not in self.joint_indices:
                continue

            target_angle = base_target

            # Add balance corrections
            if 'hip_pitch' in joint_name:
                target_angle += self.hip_pitch_correction

            if 'ankle' in joint_name:
                if 'pitch' in joint_name or '02' in joint_name:
                    target_angle += self.ankle_pitch_correction

            # Send POSITION command directly (NOT torque!)
            actuator_name = joint_name + '_ctrl'
            if actuator_name in self.actuator_indices:
                actuator_idx = self.actuator_indices[actuator_name]
                self.data.ctrl[actuator_idx] = target_angle

    def update_phase(self):
        """Update walking phase."""
        self.walking_phase += self.dt / self.step_duration

        if self.walking_phase >= 1.0:
            self.walking_phase = 0.0
            self.steps_taken += 1
            self.current_support = 'right' if self.current_support == 'left' else 'left'
            print(f"✓ Step {self.steps_taken} completed! Support: {self.current_support}")

    def run(self, duration: float = 60.0):
        """Run simulation with incremental walking."""
        print(f"\n{'='*60}")
        print(f"ZMP INCREMENTAL WALKING - robot.mjcf")
        print(f"{'='*60}")
        print(f"Phase 1: Stand still 10s (validate stability)")
        print(f"Phase 2: Walk {self.target_steps} TINY steps (1cm forward, 5mm lift)")
        print(f"Step params: {self.step_forward*100:.1f}cm fwd, {self.step_lift*100:.1f}cm lift, {self.step_duration}s/step")
        print(f"{'='*60}\n")

        start_time = time.time()
        standing_phase_duration = 10.0  # Stand still for 10s first

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                step_start = time.time()
                elapsed = step_start - start_time

                # Control strategy
                if elapsed < standing_phase_duration:
                    # Phase 1: Standing only (validate stability)
                    self.compute_pd_control(walking_enabled=False)
                else:
                    # Phase 2: Walking
                    self.compute_pd_control(walking_enabled=True)
                    if self.steps_taken < self.target_steps:
                        self.update_phase()

                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                # Print status
                if int(elapsed * 2) != int((elapsed - self.dt) * 2):  # Every 0.5s
                    base_h = self.data.qpos[2]
                    com = self.compute_com()
                    error_x, error_y = self.compute_com_error()

                    status = "STANDING" if elapsed < standing_phase_duration else f"WALKING ({self.current_support})"
                    phase_name = "---" if elapsed < standing_phase_duration else \
                                 ("SHIFT" if self.walking_phase < 0.4 else \
                                  "SWING" if self.walking_phase < 0.9 else "LAND")

                    stable = "✓" if base_h > 0.75 else "✗"

                    print(f"T:{elapsed:5.1f}s | {stable} {status:15s} | Step:{self.steps_taken}/{self.target_steps} | "
                          f"Phase:{self.walking_phase:.2f}({phase_name}) | "
                          f"H:{base_h:.3f}m | COM:({com[0]:6.3f},{com[1]:6.3f}) | "
                          f"Err:({error_x:6.3f},{error_y:6.3f})")

                if elapsed > duration:
                    break

                time_left = self.dt - (time.time() - step_start)
                if time_left > 0:
                    time.sleep(time_left)

        final_h = self.data.qpos[2]
        success = final_h > 0.75

        print(f"\n{'='*60}")
        print(f"Test finished after {elapsed:.1f}s")
        print(f"Steps completed: {self.steps_taken}/{self.target_steps}")
        print(f"Final height: {final_h:.3f}m")
        if success:
            print(f"✓✓✓ SUCCESS - Robot stayed upright!")
        else:
            print(f"✗✗✗ FAILED - Robot fell")
        print(f"{'='*60}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 zmp_walking_incremental.py <model_path> [duration]")
        sys.exit(1)

    model_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

    try:
        controller = ZMPWalkingIncremental(model_path)
        controller.run(duration)
    except KeyboardInterrupt:
        print("\n✗ Stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
