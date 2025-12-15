#!/usr/bin/env python3
"""
Shuffle Walking Controller for MuJoCo Humanoid Robot
Ultra-stable walking by keeping both feet on ground and using small weight shifts
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import sys


class ShuffleWalkingController:
    """Shuffle walking - both feet stay on ground, small alternating steps"""

    def __init__(self, model_path: str):
        """Initialize the shuffle walking controller"""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Walking parameters - VERY conservative
        self.step_length = 0.03  # 3cm steps
        self.step_duration = 2.0  # 2 seconds per step (very slow)
        self.max_hip_roll = 0.05  # Small weight shift

        # Current state
        self.walking_phase = 0.0  # 0 to 1
        self.current_moving_leg = 'right'  # which leg is sliding forward
        self.steps_taken = 0
        self.target_steps = 6

        # Base standing pose
        self.standing_pose = {
            'dof_left_hip_pitch_04': 0.02,
            'dof_left_hip_roll_03': 0.0,
            'dof_left_knee_04': 0.05,
            'dof_left_ankle_02': -0.02,
            'dof_right_hip_pitch_04': -0.02,
            'dof_right_hip_roll_03': 0.0,
            'dof_right_knee_04': -0.05,
            'dof_right_ankle_02': 0.02,
            'dof_left_shoulder_pitch_03': 0.3,
            'dof_left_elbow_02': -0.8,
            'dof_right_shoulder_pitch_03': 0.3,
            'dof_right_elbow_02': -0.8,
        }

        # Balance parameters (from successful zmp_balance_controller)
        self.com_target = np.array([0.0, 0.0])
        self.hip_correction_gain = 5.0
        self.ankle_correction_gain = 8.0
        self.hip_pitch_correction = 0.0
        self.ankle_pitch_correction = 0.0

        # Get joint and actuator indices
        self.joint_indices = {}
        self.actuator_indices = {}

        for joint_name in self.standing_pose.keys():
            try:
                joint_id = self.model.joint(joint_name).id
                self.joint_indices[joint_name] = self.model.jnt_qposadr[joint_id]
            except KeyError:
                print(f"Warning: Joint '{joint_name}' not found in model")

        for i in range(self.model.nu):
            actuator_name = self.model.actuator(i).name
            self.actuator_indices[actuator_name] = i

        print(f"✓ Found {len(self.joint_indices)} controlled joints")
        print(f"✓ Found {len(self.actuator_indices)} actuators")

    def set_initial_pose(self):
        """Set robot to standing pose"""
        for joint_name, target_angle in self.standing_pose.items():
            if joint_name in self.joint_indices:
                qpos_idx = self.joint_indices[joint_name]
                self.data.qpos[qpos_idx] = target_angle

        mujoco.mj_forward(self.model, self.data)
        print("✓ Initial standing pose set")

    def compute_com(self):
        """Compute center of mass position (x, y only)"""
        com_pos = self.data.subtree_com[0].copy()
        return com_pos[0:2]

    def compute_com_error(self):
        """Compute COM error from target"""
        current_com = self.compute_com()
        error = current_com - self.com_target
        return error

    def update_balance_corrections(self):
        """Update hip and ankle corrections based on COM error"""
        error = self.compute_com_error()
        error_x = error[0]

        # Hip strategy for large corrections
        self.hip_pitch_correction = -self.hip_correction_gain * error_x
        self.hip_pitch_correction = np.clip(self.hip_pitch_correction, -0.3, 0.3)

        # Ankle strategy for fine corrections
        self.ankle_pitch_correction = -self.ankle_correction_gain * error_x
        self.ankle_pitch_correction = np.clip(self.ankle_pitch_correction, -0.2, 0.2)

    def compute_shuffle_step(self, phase):
        """
        Compute very small forward slide of one leg
        phase: 0 to 1
        Returns: (hip_pitch_offset, hip_roll_offset)
        """
        # Smooth sine wave for gentle motion
        smooth_phase = 0.5 * (1 - np.cos(phase * np.pi))  # 0 to 1, smooth

        # Very small forward motion
        hip_pitch_offset = 0.04 * smooth_phase  # Max 4 degrees forward

        # Small weight shift to opposite leg
        hip_roll_offset = self.max_hip_roll * np.sin(phase * np.pi)

        return hip_pitch_offset, hip_roll_offset

    def compute_shuffle_pose(self):
        """Compute target pose for shuffle walking"""
        pose = self.standing_pose.copy()

        # Update balance corrections
        self.update_balance_corrections()

        # Get shuffle motion
        hip_pitch_offset, hip_roll_offset = self.compute_shuffle_step(self.walking_phase)

        if self.current_moving_leg == 'right':
            # Right leg slides forward slightly
            # Left leg supports (slightly more weight)
            pose['dof_right_hip_pitch_04'] = -0.02 - hip_pitch_offset
            pose['dof_left_hip_pitch_04'] = 0.02 + self.hip_pitch_correction

            # Weight shift: COM to left (support)
            pose['dof_left_hip_roll_03'] = -hip_roll_offset
            pose['dof_right_hip_roll_03'] = hip_roll_offset

        else:
            # Left leg slides forward slightly
            # Right leg supports
            pose['dof_left_hip_pitch_04'] = 0.02 + hip_pitch_offset
            pose['dof_right_hip_pitch_04'] = -0.02 + self.hip_pitch_correction

            # Weight shift: COM to right (support)
            pose['dof_left_hip_roll_03'] = hip_roll_offset
            pose['dof_right_hip_roll_03'] = -hip_roll_offset

        # Apply ankle corrections to both legs
        pose['dof_left_ankle_02'] = -0.02 + self.ankle_pitch_correction
        pose['dof_right_ankle_02'] = 0.02 + self.ankle_pitch_correction

        return pose

    def send_position_commands(self, target_pose):
        """Send position commands to all actuators"""
        for joint_name, target_angle in target_pose.items():
            actuator_name = joint_name + '_ctrl'
            if actuator_name in self.actuator_indices:
                actuator_idx = self.actuator_indices[actuator_name]
                self.data.ctrl[actuator_idx] = target_angle

    def update_walking_phase(self, dt):
        """Update walking phase and switch legs when needed"""
        phase_increment = dt / self.step_duration
        self.walking_phase += phase_increment

        if self.walking_phase >= 1.0:
            # Step completed, switch legs
            self.walking_phase = 0.0
            self.steps_taken += 1

            if self.current_moving_leg == 'right':
                self.current_moving_leg = 'left'
            else:
                self.current_moving_leg = 'right'

            print(f"✓ Shuffle step {self.steps_taken} completed, switching to {self.current_moving_leg} leg")

    def run(self, duration: float = 60.0):
        """Run the shuffle walking controller"""
        print(f"\nStarting Shuffle Walking Controller...")
        print(f"Target: {self.target_steps} shuffle steps")
        print(f"Step duration: {self.step_duration}s (very slow for stability)")
        print("Strategy: Both feet stay on ground, small alternating slides\n")

        # Set initial pose
        self.set_initial_pose()

        # Wait 3 seconds in standing pose before walking
        print("Standing for 3 seconds before shuffle walking...")
        start_time = time.time()
        standing_duration = 3.0

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                step_start = time.time()
                elapsed = step_start - start_time

                if elapsed < standing_duration:
                    # Standing phase with balance
                    target_pose = self.standing_pose.copy()
                    self.update_balance_corrections()

                    # Apply balance corrections
                    target_pose['dof_left_hip_pitch_04'] += self.hip_pitch_correction
                    target_pose['dof_left_ankle_02'] += self.ankle_pitch_correction
                    target_pose['dof_right_hip_pitch_04'] -= self.hip_pitch_correction
                    target_pose['dof_right_ankle_02'] += self.ankle_pitch_correction

                elif self.steps_taken < self.target_steps:
                    # Shuffle walking phase
                    target_pose = self.compute_shuffle_pose()
                    self.update_walking_phase(self.model.opt.timestep)

                else:
                    # Finished walking, return to standing
                    target_pose = self.standing_pose.copy()
                    self.update_balance_corrections()
                    target_pose['dof_left_hip_pitch_04'] += self.hip_pitch_correction
                    target_pose['dof_right_hip_pitch_04'] -= self.hip_pitch_correction

                # Send commands
                self.send_position_commands(target_pose)

                # Step simulation
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                # Print status every second
                if int(elapsed) != int(elapsed - self.model.opt.timestep):
                    base_height = self.data.qpos[2]
                    com = self.compute_com()
                    print(f"Time: {elapsed:.1f}s | Steps: {self.steps_taken}/{self.target_steps} | "
                          f"Phase: {self.walking_phase:.2f} | Moving: {self.current_moving_leg} | "
                          f"Base: {base_height:.3f}m | COM: ({com[0]:.3f}, {com[1]:.3f})")

                # Exit if duration exceeded
                if elapsed > duration:
                    break

                # Timing
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

        print(f"\n✓ Shuffle walking controller finished after {elapsed:.1f}s")
        print(f"✓ Completed {self.steps_taken}/{self.target_steps} shuffle steps")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 shuffle_walking_controller.py <model_path> [duration]")
        sys.exit(1)

    model_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

    print("=" * 60)
    print("SHUFFLE WALKING CONTROLLER - Ultra Stable")
    print("=" * 60)
    print(f"Model: {model_path}")
    print(f"Duration: {duration}s")
    print("=" * 60)

    try:
        controller = ShuffleWalkingController(model_path)
        controller.run(duration)
    except KeyboardInterrupt:
        print("\n✗ Controller stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
