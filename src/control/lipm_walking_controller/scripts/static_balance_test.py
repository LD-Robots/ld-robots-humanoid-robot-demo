#!/usr/bin/env python3
"""
Static Balance Test - robot.mjcf
Just stand still with MAXIMUM stability - no walking!
Test if we can keep it standing for 60+ seconds
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import sys


class StaticBalanceTest:
    """Ultra-conservative standing controller - NO walking"""

    def __init__(self, model_path: str):
        """Initialize controller"""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # VERY AGGRESSIVE balance gains
        self.com_kp = 100.0  # Much higher than before
        self.com_kd = 30.0

        # Standing pose (proven from zmp_balance_controller)
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

        # Target COM (try to keep it at 0, 0)
        self.target_com = np.array([0.0, 0.0])

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
        """Get COM position (x, y)"""
        return self.data.subtree_com[0][0:2].copy()

    def compute_com_velocity(self):
        """Get COM velocity (vx, vy)"""
        # Approximate COM velocity from base velocity
        return self.data.qvel[0:2].copy()

    def compute_balance_corrections(self):
        """Compute AGGRESSIVE balance corrections"""
        com = self.compute_com()
        com_vel = self.compute_com_velocity()

        error = com - self.target_com

        # PD control on COM
        correction = -self.com_kp * error - self.com_kd * com_vel

        # Apply to hip pitch (sagittal) and hip roll (frontal)
        hip_pitch_corr = correction[0]  # Forward/backward
        hip_roll_corr = correction[1]   # Left/right
        ankle_pitch_corr = correction[0] * 1.5  # Ankles help with pitch

        # Clip to reasonable ranges
        hip_pitch_corr = np.clip(hip_pitch_corr, -0.3, 0.3)
        hip_roll_corr = np.clip(hip_roll_corr, -0.3, 0.3)
        ankle_pitch_corr = np.clip(ankle_pitch_corr, -0.2, 0.2)

        return hip_pitch_corr, hip_roll_corr, ankle_pitch_corr

    def compute_standing_pose(self):
        """Compute standing pose with aggressive balance"""
        pose = self.standing_pose.copy()

        hip_pitch, hip_roll, ankle_pitch = self.compute_balance_corrections()

        # Apply corrections to BOTH legs (double support)
        pose['dof_left_hip_pitch_04'] = 0.02 + hip_pitch
        pose['dof_right_hip_pitch_04'] = -0.02 + hip_pitch

        pose['dof_left_hip_roll_03'] = 0.0 + hip_roll
        pose['dof_right_hip_roll_03'] = 0.0 - hip_roll  # Opposite for right

        pose['dof_left_ankle_02'] = -0.02 + ankle_pitch
        pose['dof_right_ankle_02'] = 0.02 - ankle_pitch

        return pose

    def send_commands(self, pose):
        """Send position commands"""
        for joint_name, angle in pose.items():
            actuator_name = joint_name + '_ctrl'
            if actuator_name in self.actuator_indices:
                self.data.ctrl[self.actuator_indices[actuator_name]] = angle

    def run(self, duration=120.0):
        """Run static balance test"""
        print(f"\n{'='*60}")
        print(f"STATIC BALANCE TEST - robot.mjcf (36.72kg)")
        print(f"{'='*60}")
        print(f"Goal: Stand still for {duration}s without falling")
        print(f"COM gains: kp={self.com_kp}, kd={self.com_kd}")
        print(f"NO WALKING - just standing!\n")

        self.set_initial_pose()

        start_time = time.time()
        max_com_error = 0.0

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                step_start = time.time()
                elapsed = step_start - start_time

                # Compute standing pose with aggressive balance
                pose = self.compute_standing_pose()
                self.send_commands(pose)

                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                # Monitor status
                if int(elapsed * 2) != int((elapsed - self.model.opt.timestep) * 2):  # Every 0.5s
                    base_h = self.data.qpos[2]
                    com = self.compute_com()
                    com_vel = self.compute_com_velocity()
                    com_error = np.linalg.norm(com - self.target_com)
                    max_com_error = max(max_com_error, com_error)

                    status = "✓ STABLE" if base_h > 0.7 else "✗ FALLING"

                    print(f"T:{elapsed:5.1f}s | H:{base_h:.3f}m | {status} | "
                          f"COM:({com[0]:6.3f},{com[1]:6.3f}) | "
                          f"err:{com_error:.4f}m | max_err:{max_com_error:.4f}m | "
                          f"vel:({com_vel[0]:5.2f},{com_vel[1]:5.2f})")

                if elapsed > duration:
                    break

                time_left = self.model.opt.timestep - (time.time() - step_start)
                if time_left > 0:
                    time.sleep(time_left)

        final_h = self.data.qpos[2]
        success = final_h > 0.7

        print(f"\n{'='*60}")
        print(f"✓ Test finished after {elapsed:.1f}s")
        print(f"Final height: {final_h:.3f}m")
        print(f"Max COM error: {max_com_error:.4f}m")
        if success:
            print(f"✓✓✓ SUCCESS - Robot stayed standing!")
        else:
            print(f"✗✗✗ FAILED - Robot fell (H < 0.7m)")
        print(f"{'='*60}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 static_balance_test.py <model_path> [duration]")
        sys.exit(1)

    model_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0

    try:
        controller = StaticBalanceTest(model_path)
        controller.run(duration)
    except KeyboardInterrupt:
        print("\n✗ Stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
