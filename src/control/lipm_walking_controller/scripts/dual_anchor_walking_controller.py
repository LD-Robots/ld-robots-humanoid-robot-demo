#!/usr/bin/env python3
"""
Dual Anchor Point Walking Controller for robot.mjcf
Menține 2 puncte de echilibru (unul pe fiecare picior) și ghidează COM-ul constant
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import sys


class DualAnchorWalkingController:
    """Walking cu 2 puncte de ancorare pentru echilibru constant"""

    def __init__(self, model_path: str):
        """Initialize controller"""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Parametri walking FOARTE conservativi
        self.step_duration = 5.0  # 5 secunde per pas
        self.step_forward = 0.02  # Doar 2cm înainte

        # DUAL ANCHOR POINTS - cele 2 puncte de echilibru
        self.left_anchor = np.array([0.0, 0.06])   # Piciorul stâng (x, y)
        self.right_anchor = np.array([0.0, -0.06]) # Piciorul drept (x, y)

        # COM target calculat dinamic între cele 2 anchor points
        self.com_target = (self.left_anchor + self.right_anchor) / 2.0  # Mijloc

        # State
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

        # Balance gains - FOARTE MARI pentru tracking agresiv al anchor points
        self.anchor_tracking_gain = 15.0  # Foarte mare pentru tracking puternic
        self.com_stabilization_gain = 10.0

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
        print(f"✓ Left anchor point: ({self.left_anchor[0]:.3f}, {self.left_anchor[1]:.3f})")
        print(f"✓ Right anchor point: ({self.right_anchor[0]:.3f}, {self.right_anchor[1]:.3f})")
        print(f"✓ COM target (center): ({self.com_target[0]:.3f}, {self.com_target[1]:.3f})")

    def set_initial_pose(self):
        """Set initial pose"""
        for joint_name, target_angle in self.standing_pose.items():
            if joint_name in self.joint_indices:
                qpos_idx = self.joint_indices[joint_name]
                self.data.qpos[qpos_idx] = target_angle
        mujoco.mj_forward(self.model, self.data)
        print("✓ Initial pose set")

    def compute_com(self):
        """Get COM position (x, y)"""
        return self.data.subtree_com[0][0:2].copy()

    def get_foot_positions(self):
        """Get current foot positions from simulation"""
        left_foot_pos = None
        right_foot_pos = None

        for i in range(self.model.ngeom):
            geom_name = self.model.geom(i).name
            if 'LFoot' in geom_name and 'collision' in geom_name:
                left_foot_pos = self.data.geom_xpos[i][0:2].copy()
            elif 'RFoot' in geom_name and 'collision' in geom_name:
                right_foot_pos = self.data.geom_xpos[i][0:2].copy()

        return left_foot_pos, right_foot_pos

    def update_anchor_points(self):
        """
        Update anchor points based on current foot positions
        Anchor points se deplasează cu picioarele!
        """
        left_foot, right_foot = self.get_foot_positions()

        if left_foot is not None:
            self.left_anchor = left_foot.copy()
        if right_foot is not None:
            self.right_anchor = right_foot.copy()

        # COM target = mijlocul între cele 2 anchor points
        self.com_target = (self.left_anchor + self.right_anchor) / 2.0

    def compute_anchor_corrections(self):
        """
        Compute corrections to keep COM centered between anchor points
        Aceasta e CHEIA - forțăm COM-ul să rămână între punctele de ancorare
        """
        current_com = self.compute_com()

        # Eroare față de target (centrul între anchor points)
        com_error = current_com - self.com_target

        # Calculăm corecții pentru fiecare direcție
        # X direction (înainte/înapoi)
        hip_pitch_correction = -self.anchor_tracking_gain * com_error[0]
        hip_pitch_correction = np.clip(hip_pitch_correction, -0.15, 0.15)

        ankle_pitch_correction = -self.com_stabilization_gain * com_error[0]
        ankle_pitch_correction = np.clip(ankle_pitch_correction, -0.10, 0.10)

        # Y direction (lateral)
        hip_roll_correction = -self.anchor_tracking_gain * com_error[1]
        hip_roll_correction = np.clip(hip_roll_correction, -0.15, 0.15)

        return hip_pitch_correction, ankle_pitch_correction, hip_roll_correction

    def compute_walking_pose(self):
        """
        Compute walking pose cu anchor point tracking constant
        """
        pose = self.standing_pose.copy()

        # Update anchor points la poziția curentă a picioarelor
        self.update_anchor_points()

        # Compute corrections pentru a menține COM între anchor points
        hip_pitch_corr, ankle_pitch_corr, hip_roll_corr = self.compute_anchor_corrections()

        phase = self.walking_phase

        # Faze:
        # 0.0 - 0.4: Weight shift (40%)
        # 0.4 - 0.9: Swing (50%)
        # 0.9 - 1.0: Landing (10%)

        if phase < 0.4:
            # WEIGHT SHIFT - mută COM spre support leg anchor point
            shift_progress = phase / 0.4

            if self.current_support == 'left':
                # Mută COM spre left anchor
                target_shift = 0.08 * shift_progress
                pose['dof_left_hip_roll_03'] = -target_shift + hip_roll_corr
                pose['dof_right_hip_roll_03'] = target_shift - hip_roll_corr
            else:
                # Mută COM spre right anchor
                target_shift = 0.08 * shift_progress
                pose['dof_left_hip_roll_03'] = target_shift - hip_roll_corr
                pose['dof_right_hip_roll_03'] = -target_shift + hip_roll_corr

            # Apply anchor corrections pe ambele picioare
            pose['dof_left_hip_pitch_04'] = 0.02 + hip_pitch_corr
            pose['dof_left_ankle_02'] = -0.02 + ankle_pitch_corr
            pose['dof_right_hip_pitch_04'] = -0.02 + hip_pitch_corr
            pose['dof_right_ankle_02'] = 0.02 + ankle_pitch_corr

        elif phase < 0.9:
            # SWING - ridicare MINIMĂ picior, COM rămâne STRICT peste support anchor
            swing_progress = (phase - 0.4) / 0.5

            # Calculăm mișcare FOARTE mică înainte
            forward_offset = self.step_forward * swing_progress
            lift = 0.008 * np.sin(swing_progress * np.pi)  # Max 8mm lift!

            # Convert la joint angles (IK simplificat)
            hip_pitch_swing = forward_offset / 0.5
            knee_swing = lift * 2.5  # Bend minimal

            if self.current_support == 'left':
                # Left supports - COM FIXAT peste left anchor!
                pose['dof_left_hip_roll_03'] = -0.08 + hip_roll_corr * 2  # Dublu gain pe support
                pose['dof_right_hip_roll_03'] = 0.08

                # Support leg - ANCHORED cu corecții puternice
                pose['dof_left_hip_pitch_04'] = 0.02 + hip_pitch_corr
                pose['dof_left_knee_04'] = 0.05
                pose['dof_left_ankle_02'] = -0.02 + ankle_pitch_corr

                # Swing leg - mișcare MINIMĂ
                pose['dof_right_hip_pitch_04'] = -(0.02 + hip_pitch_swing)
                pose['dof_right_knee_04'] = -(0.05 + knee_swing)
                pose['dof_right_ankle_02'] = 0.02 - hip_pitch_swing * 0.5
            else:
                # Right supports - COM FIXAT peste right anchor!
                pose['dof_left_hip_roll_03'] = 0.08
                pose['dof_right_hip_roll_03'] = -0.08 + hip_roll_corr * 2

                # Support leg - ANCHORED
                pose['dof_right_hip_pitch_04'] = -0.02 + hip_pitch_corr
                pose['dof_right_knee_04'] = -0.05
                pose['dof_right_ankle_02'] = 0.02 + ankle_pitch_corr

                # Swing leg - mișcare MINIMĂ
                pose['dof_left_hip_pitch_04'] = 0.02 + hip_pitch_swing
                pose['dof_left_knee_04'] = 0.05 + knee_swing
                pose['dof_left_ankle_02'] = -0.02 - hip_pitch_swing * 0.5

        else:
            # LANDING - revine la double support, COM revine la centru
            landing_progress = (phase - 0.9) / 0.1
            lateral_shift = 0.08 * (1.0 - landing_progress)

            if self.current_support == 'left':
                pose['dof_left_hip_roll_03'] = -lateral_shift + hip_roll_corr
                pose['dof_right_hip_roll_03'] = lateral_shift - hip_roll_corr
            else:
                pose['dof_left_hip_roll_03'] = lateral_shift - hip_roll_corr
                pose['dof_right_hip_roll_03'] = -lateral_shift + hip_roll_corr

            # Ambele picioare pe jos cu corecții
            pose['dof_left_hip_pitch_04'] = 0.02 + hip_pitch_corr
            pose['dof_right_hip_pitch_04'] = -0.02 + hip_pitch_corr
            pose['dof_left_ankle_02'] = -0.02 + ankle_pitch_corr
            pose['dof_right_ankle_02'] = 0.02 + ankle_pitch_corr

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
            print(f"✓ Step {self.steps_taken} completed! Anchor points updated, switching to {self.current_support}")

    def run(self, duration=50.0):
        """Run dual anchor walking controller"""
        print(f"\n{'='*70}")
        print(f"DUAL ANCHOR POINT WALKING CONTROLLER - robot.mjcf")
        print(f"{'='*70}")
        print(f"Strategie: 2 puncte de ancorare (unul pe fiecare picior)")
        print(f"COM ghidat CONSTANT să rămână între cele 2 anchor points")
        print(f"Step: {self.step_forward*100:.1f}cm forward, 8mm lift MAX")
        print(f"Duration: {self.step_duration}s per step (ultra-slow)\n")

        self.set_initial_pose()

        print("Standing for 5 seconds (calibrating anchor points)...")
        start_time = time.time()
        standing_duration = 5.0

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                step_start = time.time()
                elapsed = step_start - start_time

                if elapsed < standing_duration:
                    # Standing - calibrate anchor points
                    self.update_anchor_points()
                    hip_pitch_corr, ankle_pitch_corr, hip_roll_corr = self.compute_anchor_corrections()

                    pose = self.standing_pose.copy()
                    pose['dof_left_hip_pitch_04'] += hip_pitch_corr
                    pose['dof_right_hip_pitch_04'] += hip_pitch_corr
                    pose['dof_left_ankle_02'] += ankle_pitch_corr
                    pose['dof_right_ankle_02'] += ankle_pitch_corr
                    pose['dof_left_hip_roll_03'] += hip_roll_corr
                    pose['dof_right_hip_roll_03'] -= hip_roll_corr

                elif self.steps_taken < self.target_steps:
                    # Walking with anchor tracking
                    pose = self.compute_walking_pose()
                    self.update_phase(self.model.opt.timestep)

                else:
                    # Finished - return to standing with anchor tracking
                    self.update_anchor_points()
                    hip_pitch_corr, ankle_pitch_corr, hip_roll_corr = self.compute_anchor_corrections()

                    pose = self.standing_pose.copy()
                    pose['dof_left_hip_pitch_04'] += hip_pitch_corr
                    pose['dof_right_hip_pitch_04'] += hip_pitch_corr
                    pose['dof_left_hip_roll_03'] += hip_roll_corr
                    pose['dof_right_hip_roll_03'] -= hip_roll_corr

                self.send_commands(pose)
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                # Print status every 0.5s
                if int(elapsed * 2) != int((elapsed - self.model.opt.timestep) * 2):
                    base_h = self.data.qpos[2]
                    com = self.compute_com()
                    base_x = self.data.qpos[0]

                    # Distance from COM to center of anchors
                    com_dist_from_center = np.linalg.norm(com - self.com_target)

                    phase_name = "SHIFT" if self.walking_phase < 0.4 else \
                                 "SWING" if self.walking_phase < 0.9 else "LAND"

                    print(f"T:{elapsed:5.1f}s | Step:{self.steps_taken}/{self.target_steps} | "
                          f"P:{self.walking_phase:.2f}({phase_name:5s}) | {self.current_support:5s} | "
                          f"H:{base_h:.3f}m | COM:({com[0]:6.3f},{com[1]:6.3f}) | "
                          f"Dist:{com_dist_from_center:.4f}m | "
                          f"Anchors:L({self.left_anchor[0]:.3f},{self.left_anchor[1]:.3f})"
                          f"R({self.right_anchor[0]:.3f},{self.right_anchor[1]:.3f})")

                if elapsed > duration:
                    break

                time_left = self.model.opt.timestep - (time.time() - step_start)
                if time_left > 0:
                    time.sleep(time_left)

        total_forward = self.data.qpos[0]
        print(f"\n{'='*70}")
        print(f"✓ Finished after {elapsed:.1f}s")
        print(f"✓ Completed {self.steps_taken}/{self.target_steps} steps")
        print(f"✓ Total forward: {total_forward:.3f}m ({total_forward*100:.1f}cm)")
        print(f"✓ Final anchor points: L{self.left_anchor}, R{self.right_anchor}")
        print(f"{'='*70}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 dual_anchor_walking_controller.py <model_path> [duration]")
        sys.exit(1)

    model_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0

    try:
        controller = DualAnchorWalkingController(model_path)
        controller.run(duration)
    except KeyboardInterrupt:
        print("\n✗ Stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
