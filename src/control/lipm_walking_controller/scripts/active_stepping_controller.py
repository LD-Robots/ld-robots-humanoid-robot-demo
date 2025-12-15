#!/usr/bin/env python3
"""
Active Stepping Walking Controller for MuJoCo Humanoid Robot
Implements walking with dynamic balance using position control and COM/ZMP feedback
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import sys


class ActiveSteppingController:
    """Walking controller using active stepping strategy"""

    def __init__(self, model_path: str):
        """Initialize the walking controller"""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Walking parameters - REDUCED for stability
        self.step_length = 0.08  # meters (reduced from 0.15)
        self.step_height = 0.02  # meters (reduced from 0.05)
        self.step_duration = 1.2  # seconds per step (increased from 0.8)
        self.double_support_ratio = 0.3  # 30% double support (increased from 20%)

        # Current state
        self.walking_phase = 0.0  # 0 to 1
        self.current_support_leg = 'left'  # 'left' or 'right'
        self.steps_taken = 0
        self.target_steps = 10

        # Standing pose (copied from zmp_balance_controller)
        self.standing_pose = {
            'dof_left_hip_pitch_04': 0.02,
            'dof_left_hip_roll_03': 0.0,
            'dof_left_knee_04': 0.05,
            'dof_left_ankle_02': -0.02,
            'dof_right_hip_pitch_04': -0.02,
            'dof_right_hip_roll_03': 0.0,
            'dof_right_knee_04': -0.05,
            'dof_right_ankle_02': 0.02,
            'dof_torso_01': 0.0,
            'dof_head_01': 0.0,
            'dof_left_shoulder_pitch_03': 0.3,
            'dof_left_shoulder_roll_02': 0.2,
            'dof_left_elbow_02': -0.8,
            'dof_left_wrist_01': 0.0,
            'dof_right_shoulder_pitch_03': 0.3,
            'dof_right_shoulder_roll_02': -0.2,
            'dof_right_elbow_02': -0.8,
            'dof_right_wrist_01': 0.0,
        }

        # Balance parameters
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

        # Forward kinematics to update positions
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
        self.hip_pitch_correction = np.clip(self.hip_pitch_correction, -0.4, 0.4)

        # Ankle strategy for fine corrections
        self.ankle_pitch_correction = -self.ankle_correction_gain * error_x
        self.ankle_pitch_correction = np.clip(self.ankle_pitch_correction, -0.3, 0.3)

    def compute_swing_leg_trajectory(self, phase):
        """
        Compute swing leg joint angles for current phase - MINIMAL swing
        phase: 0 to 1 (0=start of swing, 1=end of swing)
        Returns: (hip_pitch, knee, ankle_pitch, hip_roll) angles for lateral weight shift
        """
        # VERY small swing amplitude for stability
        if phase < 0.5:
            # Lift phase (0 to 0.5) - minimal lift
            normalized = phase * 2.0  # 0 to 1
            hip_pitch = 0.02 + 0.12 * np.sin(normalized * np.pi)  # Reduced from 0.3
            knee = 0.05 + 0.25 * np.sin(normalized * np.pi)  # Reduced from 0.6
            ankle_pitch = -0.02 + 0.05 * np.sin(normalized * np.pi)  # Reduced from 0.1
        else:
            # Lower phase (0.5 to 1.0)
            normalized = (phase - 0.5) * 2.0  # 0 to 1
            hip_pitch = 0.14 - 0.12 * normalized  # Adjusted
            knee = 0.30 - 0.25 * normalized  # Adjusted
            ankle_pitch = 0.03 - 0.05 * normalized

        # Small lateral weight shift (hip roll)
        hip_roll = 0.08 * np.sin(phase * np.pi)  # Shift weight to support leg

        return hip_pitch, knee, ankle_pitch, hip_roll

    def compute_support_leg_angles(self):
        """Compute support leg angles (nearly straight with balance corrections)"""
        hip_pitch = 0.02 + self.hip_pitch_correction
        knee = 0.05
        ankle_pitch = -0.02 + self.ankle_pitch_correction
        return hip_pitch, knee, ankle_pitch

    def compute_walking_pose(self):
        """Compute target pose for current walking phase"""
        pose = self.standing_pose.copy()

        # Update balance corrections
        self.update_balance_corrections()

        if self.current_support_leg == 'left':
            # Left leg is support, right leg is swinging
            left_hip, left_knee, left_ankle = self.compute_support_leg_angles()
            right_hip, right_knee, right_ankle, right_hip_roll = self.compute_swing_leg_trajectory(self.walking_phase)

            # Apply to pose (note: right side has opposite sign conventions)
            pose['dof_left_hip_pitch_04'] = left_hip
            pose['dof_left_hip_roll_03'] = -right_hip_roll  # Shift weight to left (support)
            pose['dof_left_knee_04'] = left_knee
            pose['dof_left_ankle_02'] = left_ankle

            pose['dof_right_hip_pitch_04'] = -right_hip
            pose['dof_right_hip_roll_03'] = right_hip_roll
            pose['dof_right_knee_04'] = -right_knee
            pose['dof_right_ankle_02'] = right_ankle

        else:
            # Right leg is support, left leg is swinging
            right_hip, right_knee, right_ankle = self.compute_support_leg_angles()
            left_hip, left_knee, left_ankle, left_hip_roll = self.compute_swing_leg_trajectory(self.walking_phase)

            pose['dof_left_hip_pitch_04'] = left_hip
            pose['dof_left_hip_roll_03'] = left_hip_roll
            pose['dof_left_knee_04'] = left_knee
            pose['dof_left_ankle_02'] = left_ankle

            pose['dof_right_hip_pitch_04'] = -right_hip
            pose['dof_right_hip_roll_03'] = -left_hip_roll  # Shift weight to right (support)
            pose['dof_right_knee_04'] = -right_knee
            pose['dof_right_ankle_02'] = right_ankle

        # Keep arms swinging opposite to legs
        if self.current_support_leg == 'left':
            # Right leg forward -> left arm forward
            pose['dof_left_shoulder_pitch_03'] = 0.3 + 0.3 * self.walking_phase
            pose['dof_right_shoulder_pitch_03'] = 0.3 - 0.3 * self.walking_phase
        else:
            # Left leg forward -> right arm forward
            pose['dof_right_shoulder_pitch_03'] = 0.3 + 0.3 * self.walking_phase
            pose['dof_left_shoulder_pitch_03'] = 0.3 - 0.3 * self.walking_phase

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

            if self.current_support_leg == 'left':
                self.current_support_leg = 'right'
            else:
                self.current_support_leg = 'left'

            print(f"Step {self.steps_taken} completed, switching to {self.current_support_leg} support")

    def run(self, duration: float = 30.0):
        """Run the walking controller"""
        print(f"\nStarting Active Stepping Walking Controller...")
        print(f"Target: {self.target_steps} steps")
        print(f"Step length: {self.step_length}m, Step duration: {self.step_duration}s")
        print("Controls: Spacebar=Pause, Ctrl+Q/ESC=Quit\n")

        # Set initial pose
        self.set_initial_pose()

        # Wait 2 seconds in standing pose before walking
        print("Standing for 2 seconds before walking...")
        start_time = time.time()
        standing_duration = 2.0

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                step_start = time.time()
                elapsed = step_start - start_time

                if elapsed < standing_duration:
                    # Standing phase
                    target_pose = self.standing_pose.copy()
                    self.update_balance_corrections()

                    # Apply balance corrections to standing pose
                    target_pose['dof_left_hip_pitch_04'] += self.hip_pitch_correction
                    target_pose['dof_left_ankle_02'] += self.ankle_pitch_correction
                    target_pose['dof_right_hip_pitch_04'] -= self.hip_pitch_correction
                    target_pose['dof_right_ankle_02'] += self.ankle_pitch_correction

                elif self.steps_taken < self.target_steps:
                    # Walking phase
                    target_pose = self.compute_walking_pose()
                    self.update_walking_phase(self.model.opt.timestep)

                else:
                    # Finished walking, return to standing
                    target_pose = self.standing_pose.copy()

                # Send commands
                self.send_position_commands(target_pose)

                # Step simulation
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                # Print status
                if int(elapsed) != int(elapsed - self.model.opt.timestep):
                    base_height = self.data.qpos[2]
                    com = self.compute_com()
                    print(f"Time: {elapsed:.1f}s | Steps: {self.steps_taken}/{self.target_steps} | "
                          f"Phase: {self.walking_phase:.2f} | Support: {self.current_support_leg} | "
                          f"Base: {base_height:.3f}m | COM: ({com[0]:.3f}, {com[1]:.3f})")

                # Exit if duration exceeded
                if elapsed > duration:
                    break

                # Simple timing
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

        print(f"\n✓ Walking controller finished after {elapsed:.1f}s")
        print(f"✓ Completed {self.steps_taken} steps")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 active_stepping_controller.py <model_path> [duration]")
        sys.exit(1)

    model_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

    print("=" * 60)
    print("ACTIVE STEPPING WALKING CONTROLLER")
    print("=" * 60)
    print(f"Model: {model_path}")
    print(f"Duration: {duration}s")
    print("=" * 60)

    try:
        controller = ActiveSteppingController(model_path)
        controller.run(duration)
    except KeyboardInterrupt:
        print("\n✗ Controller stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
