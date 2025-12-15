#!/usr/bin/env python3
"""
Forward Walking Controller for robot.mjcf (URDF local)
Ultra-conservative forward stepping with COM stabilization
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import sys


class ForwardWalkingRobot:
    """Forward walking controller optimized for robot.mjcf"""

    def __init__(self, model_path: str):
        """Initialize controller"""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Ultra-conservative walking parameters for 36kg robot
        self.step_duration = 4.0  # 4 seconds per step (VERY slow)
        self.step_forward_distance = 0.03  # Only 3cm forward per step
        self.step_lift_height = 0.01  # Only 1cm lift (minimal)
        self.weight_shift_duration = 1.5  # 1.5s for weight shift before lift

        # Phase breakdown (0 to 1):
        # 0.0 - 0.375: Weight shift to support leg (1.5s / 4s = 37.5%)
        # 0.375 - 0.875: Swing phase with minimal lift (2s / 4s = 50%)
        # 0.875 - 1.0: Landing and stabilization (0.5s / 4s = 12.5%)

        # State
        self.walking_phase = 0.0
        self.current_support = 'left'
        self.steps_taken = 0
        self.target_steps = 6

        # Standing pose (from successful zmp_balance_controller)
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

        # Balance parameters (proven to work)
        self.com_target = np.array([0.0, 0.0])
        self.hip_correction_gain = 5.0
        self.ankle_correction_gain = 8.0
        self.hip_pitch_correction = 0.0
        self.ankle_pitch_correction = 0.0

        # Weight shift parameters
        self.max_lateral_shift = 0.10  # 10 degrees hip roll shift

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

    def set_initial_pose(self):
        """Set initial standing pose"""
        for joint_name, target_angle in self.standing_pose.items():
            if joint_name in self.joint_indices:
                qpos_idx = self.joint_indices[joint_name]
                self.data.qpos[qpos_idx] = target_angle
        mujoco.mj_forward(self.model, self.data)
        print("✓ Initial pose set")

    def compute_com(self):
        """Get COM position"""
        return self.data.subtree_com[0][0:2].copy()

    def compute_com_error(self):
        """Compute COM error from target"""
        return self.compute_com() - self.com_target

    def update_balance_corrections(self):
        """Update balance corrections using proven gains"""
        error = self.compute_com_error()
        self.hip_pitch_correction = -self.hip_correction_gain * error[0]
        self.hip_pitch_correction = np.clip(self.hip_pitch_correction, -0.15, 0.15)

        self.ankle_pitch_correction = -self.ankle_correction_gain * error[0]
        self.ankle_pitch_correction = np.clip(self.ankle_pitch_correction, -0.10, 0.10)

    def compute_forward_step_trajectory(self, phase):
        """
        Compute FORWARD step trajectory (pași înainte, nu lateral!)
        phase: 0 to 1 during swing
        Returns: (forward_offset, lift_height)
        """
        # Ultra-smooth trajectory
        if phase < 0.5:
            # First half: lift and move forward
            t = phase * 2.0
            smooth = 0.5 * (1 - np.cos(t * np.pi))  # Smooth 0 to 1

            forward = self.step_forward_distance * smooth
            lift = self.step_lift_height * np.sin(t * np.pi)
        else:
            # Second half: continue forward, lower down
            t = (phase - 0.5) * 2.0
            smooth_forward = 0.5 + 0.5 * (1 - np.cos(t * np.pi))  # 0.5 to 1

            forward = self.step_forward_distance * smooth_forward
            lift = self.step_lift_height * np.sin((0.5 + t * 0.5) * np.pi)

        return forward, lift

    def forward_to_joint_angles(self, forward_offset, lift_height):
        """
        Convert forward offset and lift to joint angles (simplified IK)
        forward_offset: meters forward
        lift_height: meters up
        Returns: (hip_pitch, knee, ankle_pitch)
        """
        # Approximate leg length
        thigh_length = 0.25  # ~25cm
        shin_length = 0.25   # ~25cm

        # Hip pitch: forward motion
        hip_pitch = forward_offset / (thigh_length + shin_length)  # Radians

        # Knee: bend for lift (need to shorten leg)
        leg_shortening = lift_height
        # Use small-angle approximation: knee bend ≈ 2 * shortening / leg_length
        knee_bend = 2.0 * leg_shortening / (thigh_length + shin_length)
        knee = 0.05 + knee_bend  # Base knee + additional bend

        # Ankle: compensate hip pitch to keep foot flat
        ankle_pitch = -hip_pitch * 0.7

        return hip_pitch, knee, ankle_pitch

    def compute_walking_pose(self):
        """Compute walking pose with forward steps"""
        pose = self.standing_pose.copy()
        self.update_balance_corrections()

        phase = self.walking_phase
        weight_shift_phase = 0.375  # 37.5% for weight shift
        swing_end_phase = 0.875     # 87.5% swing ends

        if phase < weight_shift_phase:
            # PHASE 1: Weight shift to support leg (NO movement yet)
            shift_progress = phase / weight_shift_phase  # 0 to 1
            lateral_shift = self.max_lateral_shift * shift_progress

            if self.current_support == 'left':
                # Shift weight to left leg
                pose['dof_left_hip_roll_03'] = -lateral_shift
                pose['dof_right_hip_roll_03'] = lateral_shift
            else:
                # Shift weight to right leg
                pose['dof_left_hip_roll_03'] = lateral_shift
                pose['dof_right_hip_roll_03'] = -lateral_shift

            # Both legs straight, apply balance corrections to support leg only
            if self.current_support == 'left':
                pose['dof_left_hip_pitch_04'] = 0.02 + self.hip_pitch_correction
                pose['dof_left_ankle_02'] = -0.02 + self.ankle_pitch_correction
            else:
                pose['dof_right_hip_pitch_04'] = -0.02 + self.hip_pitch_correction
                pose['dof_right_ankle_02'] = 0.02 + self.ankle_pitch_correction

        elif phase < swing_end_phase:
            # PHASE 2: Swing leg FORWARD (pași înainte!)
            swing_progress = (phase - weight_shift_phase) / (swing_end_phase - weight_shift_phase)
            forward_offset, lift_height = self.compute_forward_step_trajectory(swing_progress)

            # Convert to joint angles
            swing_hip_pitch, swing_knee, swing_ankle = self.forward_to_joint_angles(
                forward_offset, lift_height
            )

            # Apply to swing leg (FORWARD motion)
            if self.current_support == 'left':
                # Left supports, right swings FORWARD
                pose['dof_left_hip_roll_03'] = -self.max_lateral_shift
                pose['dof_right_hip_roll_03'] = self.max_lateral_shift

                # Support leg (left)
                pose['dof_left_hip_pitch_04'] = 0.02 + self.hip_pitch_correction
                pose['dof_left_knee_04'] = 0.05
                pose['dof_left_ankle_02'] = -0.02 + self.ankle_pitch_correction

                # Swing leg (right) - MOVES FORWARD
                pose['dof_right_hip_pitch_04'] = -(0.02 + swing_hip_pitch)  # Negative for right side
                pose['dof_right_knee_04'] = -(0.05 + swing_knee)
                pose['dof_right_ankle_02'] = 0.02 + swing_ankle
            else:
                # Right supports, left swings FORWARD
                pose['dof_left_hip_roll_03'] = self.max_lateral_shift
                pose['dof_right_hip_roll_03'] = -self.max_lateral_shift

                # Support leg (right)
                pose['dof_right_hip_pitch_04'] = -0.02 + self.hip_pitch_correction
                pose['dof_right_knee_04'] = -0.05
                pose['dof_right_ankle_02'] = 0.02 + self.ankle_pitch_correction

                # Swing leg (left) - MOVES FORWARD
                pose['dof_left_hip_pitch_04'] = 0.02 + swing_hip_pitch
                pose['dof_left_knee_04'] = 0.05 + swing_knee
                pose['dof_left_ankle_02'] = -0.02 + swing_ankle

        else:
            # PHASE 3: Landing and return to double support
            landing_progress = (phase - swing_end_phase) / (1.0 - swing_end_phase)
            lateral_shift = self.max_lateral_shift * (1.0 - landing_progress)

            if self.current_support == 'left':
                pose['dof_left_hip_roll_03'] = -lateral_shift
                pose['dof_right_hip_roll_03'] = lateral_shift
            else:
                pose['dof_left_hip_roll_03'] = lateral_shift
                pose['dof_right_hip_roll_03'] = -lateral_shift

            # Return to standing with balance
            pose['dof_left_hip_pitch_04'] = 0.02 + self.hip_pitch_correction / 2
            pose['dof_right_hip_pitch_04'] = -0.02 + self.hip_pitch_correction / 2

        return pose

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
            print(f"✓ Forward step {self.steps_taken} completed! Switching to {self.current_support} support")

    def run(self, duration=50.0):
        """Run forward walking controller"""
        print(f"\n{'='*60}")
        print(f"FORWARD WALKING CONTROLLER - robot.mjcf")
        print(f"{'='*60}")
        print(f"PAȘI ÎNAINTE (FORWARD), nu lateral!")
        print(f"Step: {self.step_forward_distance*100:.1f}cm forward, {self.step_lift_height*100:.1f}cm lift")
        print(f"Duration: {self.step_duration}s per step (ultra-slow for 36kg robot)")
        print(f"Target: {self.target_steps} forward steps\n")

        self.set_initial_pose()

        print("Standing for 4 seconds before walking...")
        start_time = time.time()
        standing_duration = 4.0

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                step_start = time.time()
                elapsed = step_start - start_time

                if elapsed < standing_duration:
                    # Standing phase with balance
                    pose = self.standing_pose.copy()
                    self.update_balance_corrections()
                    pose['dof_left_hip_pitch_04'] += self.hip_pitch_correction
                    pose['dof_right_hip_pitch_04'] -= self.hip_pitch_correction
                    pose['dof_left_ankle_02'] += self.ankle_pitch_correction
                    pose['dof_right_ankle_02'] += self.ankle_pitch_correction

                elif self.steps_taken < self.target_steps:
                    # Walking phase
                    pose = self.compute_walking_pose()
                    self.update_phase(self.model.opt.timestep)

                else:
                    # Finished, return to standing
                    pose = self.standing_pose.copy()
                    self.update_balance_corrections()
                    pose['dof_left_hip_pitch_04'] += self.hip_pitch_correction
                    pose['dof_right_hip_pitch_04'] -= self.hip_pitch_correction

                self.send_commands(pose)
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                # Print status
                if int(elapsed * 2) != int((elapsed - self.model.opt.timestep) * 2):  # Every 0.5s
                    base_h = self.data.qpos[2]
                    com = self.compute_com()
                    base_x = self.data.qpos[0]  # Forward position!

                    phase_name = "SHIFT" if self.walking_phase < 0.375 else \
                                 "SWING" if self.walking_phase < 0.875 else "LAND"

                    print(f"T:{elapsed:5.1f}s | Step:{self.steps_taken}/{self.target_steps} | "
                          f"Phase:{self.walking_phase:.2f}({phase_name}) | {self.current_support:5s} | "
                          f"Fwd:{base_x:6.3f}m | H:{base_h:.3f}m | COM:({com[0]:6.3f},{com[1]:6.3f})")

                if elapsed > duration:
                    break

                time_left = self.model.opt.timestep - (time.time() - step_start)
                if time_left > 0:
                    time.sleep(time_left)

        total_forward = self.data.qpos[0]  # Total forward distance traveled
        print(f"\n{'='*60}")
        print(f"✓ Finished after {elapsed:.1f}s")
        print(f"✓ Completed {self.steps_taken}/{self.target_steps} forward steps")
        print(f"✓ Total forward distance: {total_forward:.3f}m ({total_forward*100:.1f}cm)")
        print(f"{'='*60}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 forward_walking_robot.py <model_path> [duration]")
        sys.exit(1)

    model_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0

    try:
        controller = ForwardWalkingRobot(model_path)
        controller.run(duration)
    except KeyboardInterrupt:
        print("\n✗ Stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
