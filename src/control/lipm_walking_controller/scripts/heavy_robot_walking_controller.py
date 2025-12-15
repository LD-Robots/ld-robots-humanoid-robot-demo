#!/usr/bin/env python3
"""
Heavy Robot Walking Controller for robot.mjcf
Specialized for heavy humanoid (36kg) with explicit double support phases
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import sys


class HeavyRobotWalkingController:
    """Walking controller for heavy humanoid with weight shift phases"""

    def __init__(self, model_path: str):
        """Initialize controller"""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Walking parameters - VERY slow for 36kg robot
        self.step_duration = 3.0  # 3 seconds per full step cycle
        self.double_support_ratio = 0.4  # 40% in double support (weight shift)

        # Phase breakdown:
        # 0.0 - 0.4: Double support, weight shift to support leg
        # 0.4 - 0.9: Single support, swing leg moves
        # 0.9 - 1.0: Double support, landing

        # State
        self.walking_phase = 0.0
        self.current_support = 'left'
        self.steps_taken = 0
        self.target_steps = 6

        # Standing pose
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

        # Balance parameters (from successful zmp_balance_controller)
        self.com_target = np.array([0.0, 0.0])
        self.hip_correction_gain = 5.0
        self.ankle_correction_gain = 8.0
        self.hip_pitch_correction = 0.0
        self.ankle_pitch_correction = 0.0

        # Weight shift parameters for heavy robot
        self.max_hip_roll_shift = 0.15  # 15 degrees lateral shift

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
        """Compute COM error"""
        return self.compute_com() - self.com_target

    def update_balance_corrections(self):
        """Update balance corrections"""
        error = self.compute_com_error()
        self.hip_pitch_correction = -self.hip_correction_gain * error[0]
        self.hip_pitch_correction = np.clip(self.hip_pitch_correction, -0.2, 0.2)

        self.ankle_pitch_correction = -self.ankle_correction_gain * error[0]
        self.ankle_pitch_correction = np.clip(self.ankle_pitch_correction, -0.15, 0.15)

    def compute_walking_pose(self):
        """Compute pose based on phase"""
        pose = self.standing_pose.copy()
        self.update_balance_corrections()

        phase = self.walking_phase

        if phase < self.double_support_ratio:
            # DOUBLE SUPPORT - Weight shift phase
            # Gradually shift weight to support leg
            shift_progress = phase / self.double_support_ratio  # 0 to 1
            hip_roll = self.max_hip_roll_shift * shift_progress

            if self.current_support == 'left':
                # Shifting weight to left
                pose['dof_left_hip_roll_03'] = -hip_roll
                pose['dof_right_hip_roll_03'] = hip_roll
                # Both legs straight
                pose['dof_left_hip_pitch_04'] = 0.02 + self.hip_pitch_correction
                pose['dof_right_hip_pitch_04'] = -0.02
                pose['dof_left_knee_04'] = 0.05
                pose['dof_right_knee_04'] = -0.05
            else:
                # Shifting weight to right
                pose['dof_left_hip_roll_03'] = hip_roll
                pose['dof_right_hip_roll_03'] = -hip_roll
                pose['dof_left_hip_pitch_04'] = 0.02
                pose['dof_right_hip_pitch_04'] = -0.02 + self.hip_pitch_correction
                pose['dof_left_knee_04'] = 0.05
                pose['dof_right_knee_04'] = -0.05

        elif phase < 0.9:
            # SINGLE SUPPORT - Swing phase
            swing_progress = (phase - self.double_support_ratio) / (0.9 - self.double_support_ratio)

            # Very small step - just 2cm forward, 1cm lift
            if swing_progress < 0.5:
                # Lift
                normalized = swing_progress * 2.0
                hip_pitch_swing = 0.06 * np.sin(normalized * np.pi)
                knee_swing = 0.05 + 0.15 * np.sin(normalized * np.pi)
            else:
                # Lower
                normalized = (swing_progress - 0.5) * 2.0
                hip_pitch_swing = 0.06 * np.sin((0.5 + normalized * 0.5) * np.pi)
                knee_swing = 0.20 - 0.15 * normalized

            ankle_swing = -0.02

            if self.current_support == 'left':
                # Left supports, right swings
                pose['dof_left_hip_roll_03'] = -self.max_hip_roll_shift
                pose['dof_right_hip_roll_03'] = self.max_hip_roll_shift

                pose['dof_left_hip_pitch_04'] = 0.02 + self.hip_pitch_correction
                pose['dof_left_knee_04'] = 0.05
                pose['dof_left_ankle_02'] = -0.02 + self.ankle_pitch_correction

                pose['dof_right_hip_pitch_04'] = -hip_pitch_swing
                pose['dof_right_knee_04'] = -knee_swing
                pose['dof_right_ankle_02'] = ankle_swing
            else:
                # Right supports, left swings
                pose['dof_left_hip_roll_03'] = self.max_hip_roll_shift
                pose['dof_right_hip_roll_03'] = -self.max_hip_roll_shift

                pose['dof_right_hip_pitch_04'] = -0.02 + self.hip_pitch_correction
                pose['dof_right_knee_04'] = -0.05
                pose['dof_right_ankle_02'] = 0.02 + self.ankle_pitch_correction

                pose['dof_left_hip_pitch_04'] = hip_pitch_swing
                pose['dof_left_knee_04'] = knee_swing
                pose['dof_left_ankle_02'] = ankle_swing

        else:
            # DOUBLE SUPPORT - Landing phase
            landing_progress = (phase - 0.9) / 0.1
            hip_roll = self.max_hip_roll_shift * (1 - landing_progress)

            if self.current_support == 'left':
                pose['dof_left_hip_roll_03'] = -hip_roll
                pose['dof_right_hip_roll_03'] = hip_roll
            else:
                pose['dof_left_hip_roll_03'] = hip_roll
                pose['dof_right_hip_roll_03'] = -hip_roll

            # Return to standing
            pose['dof_left_hip_pitch_04'] = 0.02 + self.hip_pitch_correction
            pose['dof_right_hip_pitch_04'] = -0.02
            pose['dof_left_knee_04'] = 0.05
            pose['dof_right_knee_04'] = -0.05

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
            print(f"✓ Step {self.steps_taken} completed, switching to {self.current_support} support")

    def run(self, duration=40.0):
        """Run controller"""
        print(f"\nStarting Heavy Robot Walking Controller...")
        print(f"Robot mass: 36.72 kg")
        print(f"Step duration: {self.step_duration}s (slow for stability)")
        print(f"Double support: {self.double_support_ratio*100:.0f}% (weight shift)\n")

        self.set_initial_pose()

        print("Standing for 3 seconds...")
        start_time = time.time()
        standing_duration = 3.0

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                step_start = time.time()
                elapsed = step_start - start_time

                if elapsed < standing_duration:
                    pose = self.standing_pose.copy()
                    self.update_balance_corrections()
                    pose['dof_left_hip_pitch_04'] += self.hip_pitch_correction
                    pose['dof_right_hip_pitch_04'] -= self.hip_pitch_correction
                elif self.steps_taken < self.target_steps:
                    pose = self.compute_walking_pose()
                    self.update_phase(self.model.opt.timestep)
                else:
                    pose = self.standing_pose.copy()

                self.send_commands(pose)
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                # Print every second
                if int(elapsed) != int(elapsed - self.model.opt.timestep):
                    base_h = self.data.qpos[2]
                    com = self.compute_com()
                    phase_name = "DOUBLE" if (self.walking_phase < self.double_support_ratio or self.walking_phase > 0.9) else "SINGLE"
                    print(f"Time: {elapsed:.1f}s | Step: {self.steps_taken}/{self.target_steps} | "
                          f"Phase: {self.walking_phase:.2f} ({phase_name}) | Support: {self.current_support} | "
                          f"Base: {base_h:.3f}m | COM: ({com[0]:.3f}, {com[1]:.3f})")

                if elapsed > duration:
                    break

                time_left = self.model.opt.timestep - (time.time() - step_start)
                if time_left > 0:
                    time.sleep(time_left)

        print(f"\n✓ Finished after {elapsed:.1f}s")
        print(f"✓ Completed {self.steps_taken}/{self.target_steps} steps")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 heavy_robot_walking_controller.py <model_path> [duration]")
        sys.exit(1)

    model_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0

    print("=" * 60)
    print("HEAVY ROBOT WALKING CONTROLLER (36kg)")
    print("=" * 60)
    print(f"Model: {model_path}")
    print(f"Duration: {duration}s")
    print("=" * 60)

    try:
        controller = HeavyRobotWalkingController(model_path)
        controller.run(duration)
    except KeyboardInterrupt:
        print("\n✗ Stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
