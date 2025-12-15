#!/usr/bin/env python3
"""
LIPM Preview Walking Controller for MuJoCo Humanoid
Based on Linear Inverted Pendulum Model with ZMP preview control
Inspired by: https://github.com/rdesarz/biped-walking-controller
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import sys
from scipy.linalg import solve_discrete_are


class LIPMPreviewController:
    """Walking controller using LIPM with preview control"""

    def __init__(self, model_path: str):
        """Initialize LIPM preview controller"""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # LIPM parameters
        self.com_height = 0.75  # CoM height (m) - adjusted for robot
        self.g = 9.81  # gravity
        self.dt = 0.01  # control timestep (10ms)
        self.preview_window = 1.6  # preview 1.6 seconds ahead
        self.N = int(self.preview_window / self.dt)  # preview steps

        # Walking parameters
        self.step_length = 0.05  # 5cm steps
        self.step_width = 0.12  # 12cm between feet
        self.step_duration = 1.5  # 1.5s per step
        self.double_support_ratio = 0.2  # 20% double support
        self.step_height = 0.02  # 2cm lift

        # State
        self.current_support = 'left'
        self.walking_phase = 0.0
        self.steps_taken = 0
        self.target_steps = 8
        self.com_state = np.zeros(3)  # [x, x_dot, x_ddot]
        self.zmp_ref_buffer = []  # Future ZMP references

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

        # Compute preview control gains
        self.compute_preview_gains()

        print(f"✓ Found {len(self.joint_indices)} controlled joints")
        print(f"✓ Found {len(self.actuator_indices)} actuators")
        print(f"✓ Preview window: {self.preview_window}s ({self.N} steps)")

    def compute_preview_gains(self):
        """Compute preview control gains using LQR"""
        # LIPM state-space model (discrete time)
        # x_k+1 = A*x_k + B*u_k
        # where x = [CoM_pos, CoM_vel, CoM_acc], u = ZMP

        omega = np.sqrt(self.g / self.com_height)
        T = self.dt

        # Continuous time: x_ddot = omega^2 * (x - zmp)
        # State: [x, x_dot, x_ddot], Input: zmp
        A_c = np.array([
            [0, 1, 0],
            [0, 0, 1],
            [omega**2, 0, 0]
        ])
        B_c = np.array([[0], [0], [-omega**2]])

        # Discretize using zero-order hold
        A = np.eye(3) + A_c * T + 0.5 * (A_c @ A_c) * T**2
        B = B_c * T + 0.5 * (A_c @ B_c) * T**2

        # Output: y = CoM_pos - ZMP = [1 0 0]*x - u
        C = np.array([[1, 0, 0]])

        # Augmented system for tracking (includes integrator)
        A_aug = np.vstack([
            np.hstack([np.eye(1), C @ A]),
            np.hstack([np.zeros((3, 1)), A])
        ])
        B_aug = np.vstack([C @ B, B])

        # LQR weights
        Q = np.diag([1e6, 1, 1, 1])  # Heavily penalize tracking error
        R = np.array([[1e-6]])  # Small input penalty

        # Solve discrete-time algebraic Riccati equation
        try:
            P = solve_discrete_are(A_aug, B_aug, Q, R)

            # Compute optimal gain
            K = np.linalg.inv(R + B_aug.T @ P @ B_aug) @ (B_aug.T @ P @ A_aug)

            self.K_fb = K[0, 1:]  # State feedback gains
            self.K_i = K[0, 0]    # Integral gain

            # Preview gains (simplified - use constant preview gain)
            self.K_p = np.ones(self.N) * 0.01

            print(f"✓ Preview gains computed: K_i={self.K_i:.3f}")
        except Exception as e:
            print(f"Warning: Could not compute optimal gains, using defaults: {e}")
            # Fallback to hand-tuned gains
            self.K_fb = np.array([100.0, 10.0, 0.1])  # [pos, vel, acc]
            self.K_i = 1000.0
            self.K_p = np.ones(self.N) * 0.01

    def generate_zmp_trajectory(self):
        """Generate reference ZMP trajectory for preview window"""
        zmp_refs = []

        for i in range(self.N):
            future_time = i * self.dt
            future_phase = self.walking_phase + future_time / self.step_duration

            # Determine which foot supports at future_phase
            step_num = int(future_phase)
            phase_in_step = future_phase - step_num

            if step_num % 2 == 0:  # Even step - left support
                support = 'left'
            else:  # Odd step - right support
                support = 'right'

            # ZMP is under support foot
            if support == 'left':
                zmp_y = self.step_width / 2
            else:
                zmp_y = -self.step_width / 2

            # ZMP moves forward with steps
            zmp_x = (step_num * self.step_length) + (phase_in_step * self.step_length)

            zmp_refs.append([zmp_x, zmp_y])

        return np.array(zmp_refs)

    def compute_com_reference(self):
        """Compute desired CoM position using preview control"""
        # Get ZMP reference trajectory
        zmp_traj = self.generate_zmp_trajectory()

        # Current state error (integral of tracking error)
        com_current = self.data.subtree_com[0][0:2]  # [x, y]
        zmp_current = zmp_traj[0]
        error_integral = com_current - zmp_current

        # State feedback
        u_fb = -self.K_fb @ self.com_state

        # Integral action
        u_i = -self.K_i * error_integral[0]  # Only X direction for now

        # Preview action (feed-forward)
        u_p = 0.0
        for i in range(min(self.N, len(zmp_traj))):
            u_p += self.K_p[i] * zmp_traj[i, 0]

        # Total control
        com_ref_x = u_fb + u_i + u_p
        com_ref_y = zmp_traj[0, 1]  # Simplified: follow ZMP in Y

        return np.array([com_ref_x, com_ref_y])

    def compute_swing_foot_trajectory(self, phase):
        """Compute swing foot trajectory"""
        if phase < 0.5:
            # Lift phase
            t = phase * 2.0
            z = self.step_height * np.sin(t * np.pi)
            x = self.step_length * 0.5 * (1 - np.cos(t * np.pi))
        else:
            # Lower phase
            t = (phase - 0.5) * 2.0
            z = self.step_height * np.sin(t * np.pi)
            x = self.step_length * (0.5 + 0.5 * (1 - np.cos(t * np.pi)))

        return x, z

    def compute_walking_pose(self):
        """Compute walking pose using LIPM control"""
        pose = self.standing_pose.copy()

        # Compute CoM reference from preview control
        com_ref = self.compute_com_reference()

        # Map CoM to hip roll for lateral balance
        com_current = self.data.subtree_com[0][0:2]
        com_error_y = com_current[1] - com_ref[1]
        hip_roll_correction = -8.0 * com_error_y  # Proportional control
        hip_roll_correction = np.clip(hip_roll_correction, -0.15, 0.15)

        # Phase-dependent leg motion
        phase = self.walking_phase

        if phase < self.double_support_ratio:
            # Double support - weight shift
            shift = phase / self.double_support_ratio
            if self.current_support == 'left':
                pose['dof_left_hip_roll_03'] = -0.08 * shift + hip_roll_correction
                pose['dof_right_hip_roll_03'] = 0.08 * shift
            else:
                pose['dof_left_hip_roll_03'] = 0.08 * shift
                pose['dof_right_hip_roll_03'] = -0.08 * shift + hip_roll_correction

        elif phase < 1.0 - self.double_support_ratio:
            # Single support - swing phase
            swing_phase = (phase - self.double_support_ratio) / (1.0 - 2*self.double_support_ratio)
            swing_x, swing_z = self.compute_swing_foot_trajectory(swing_phase)

            # Convert to joint angles (simplified IK)
            hip_pitch = swing_x / 0.4  # Approximate
            knee = max(0.1, swing_z * 3.0)  # Bend knee for clearance

            if self.current_support == 'left':
                # Left supports, right swings
                pose['dof_left_hip_roll_03'] = -0.08 + hip_roll_correction
                pose['dof_right_hip_roll_03'] = 0.08

                pose['dof_right_hip_pitch_04'] = -hip_pitch
                pose['dof_right_knee_04'] = -knee
                pose['dof_right_ankle_02'] = hip_pitch * 0.5
            else:
                # Right supports, left swings
                pose['dof_left_hip_roll_03'] = 0.08
                pose['dof_right_hip_roll_03'] = -0.08 + hip_roll_correction

                pose['dof_left_hip_pitch_04'] = hip_pitch
                pose['dof_left_knee_04'] = knee
                pose['dof_left_ankle_02'] = -hip_pitch * 0.5
        else:
            # Double support - landing
            pose['dof_left_hip_roll_03'] = hip_roll_correction / 2
            pose['dof_right_hip_roll_03'] = hip_roll_correction / 2

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

    def set_initial_pose(self):
        """Set initial pose"""
        for joint_name, target_angle in self.standing_pose.items():
            if joint_name in self.joint_indices:
                qpos_idx = self.joint_indices[joint_name]
                self.data.qpos[qpos_idx] = target_angle
        mujoco.mj_forward(self.model, self.data)
        print("✓ Initial pose set")

    def run(self, duration=40.0):
        """Run controller"""
        print(f"\n{'='*60}")
        print(f"LIPM PREVIEW WALKING CONTROLLER")
        print(f"{'='*60}")
        print(f"Using Linear Inverted Pendulum Model with preview control")
        print(f"Step length: {self.step_length}m, Duration: {self.step_duration}s")
        print(f"Preview horizon: {self.preview_window}s\n")

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
                elif self.steps_taken < self.target_steps:
                    pose = self.compute_walking_pose()
                    self.update_phase(self.dt)
                else:
                    pose = self.standing_pose.copy()

                self.send_commands(pose)
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                # Update CoM state for control
                com_pos = self.data.subtree_com[0][0:2]
                # Simplified: assume velocity and acceleration are small
                self.com_state = np.array([com_pos[0], 0.0, 0.0])

                if int(elapsed) != int(elapsed - self.model.opt.timestep):
                    base_h = self.data.qpos[2]
                    print(f"Time: {elapsed:.1f}s | Steps: {self.steps_taken}/{self.target_steps} | "
                          f"Phase: {self.walking_phase:.2f} | Support: {self.current_support} | "
                          f"Base: {base_h:.3f}m | CoM: ({com_pos[0]:.3f}, {com_pos[1]:.3f})")

                if elapsed > duration:
                    break

                time_left = self.model.opt.timestep - (time.time() - step_start)
                if time_left > 0:
                    time.sleep(time_left)

        print(f"\n✓ Finished after {elapsed:.1f}s")
        print(f"✓ Completed {self.steps_taken}/{self.target_steps} steps")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 lipm_preview_walking_controller.py <model_path> [duration]")
        sys.exit(1)

    model_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0

    try:
        controller = LIPMPreviewController(model_path)
        controller.run(duration)
    except KeyboardInterrupt:
        print("\n✗ Stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
