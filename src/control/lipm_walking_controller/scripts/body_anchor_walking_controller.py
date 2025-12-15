#!/usr/bin/env python3
"""
Body Anchor Walking Controller for robot.mjcf
Puncte de ancorare pe PELVIS și TORSO (părți rigide ale corpului)
NU pe picioare care se mișcă!
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import sys


class BodyAnchorWalkingController:
    """Walking cu puncte de ancorare pe pelvis și torso"""

    def __init__(self, model_path: str):
        """Initialize controller"""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Walking parameters - ultra-conservative
        self.step_duration = 6.0  # 6 secunde per pas!
        self.step_forward = 0.015  # Doar 1.5cm

        # ANCHOR POINTS pe corp rigid (NU pe picioare!)
        # Acestea sunt RELATIVE la poziția body-urilor
        self.pelvis_anchor_offset = np.array([0.0, 0.0])  # Pelvis center
        self.torso_anchor_offset = np.array([0.0, 0.0])   # Torso center

        # Targets dinamice calculate din poziții reale
        self.pelvis_anchor = np.array([0.0, 0.0])
        self.torso_anchor = np.array([0.0, 0.0])
        self.target_com = np.array([0.0, 0.0])  # Mijlocul între pelvis și torso

        # State
        self.walking_phase = 0.0
        self.current_support = 'left'
        self.steps_taken = 0
        self.target_steps = 3

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

        # GAINS SUPER MARI pentru tracking strict
        self.pelvis_tracking_gain = 20.0   # Tracking pelvis anchor
        self.torso_tracking_gain = 18.0    # Tracking torso anchor
        self.com_stabilization_gain = 12.0  # General COM stabilization

        # Find body IDs pentru pelvis și torso
        self.pelvis_body_id = None
        self.torso_body_id = None

        for i in range(self.model.nbody):
            body_name = self.model.body(i).name.lower()
            if 'pelvis' in body_name or 'hip' in body_name:
                if self.pelvis_body_id is None:  # Primul găsit
                    self.pelvis_body_id = i
                    print(f"✓ Found pelvis body: {self.model.body(i).name} (id={i})")
            if 'torso' in body_name or 'chest' in body_name or 'trunk' in body_name:
                if self.torso_body_id is None:
                    self.torso_body_id = i
                    print(f"✓ Found torso body: {self.model.body(i).name} (id={i})")

        # Fallback: use body 0 (root) pentru pelvis, body 1 pentru torso
        if self.pelvis_body_id is None:
            self.pelvis_body_id = 0
            print(f"⚠ Using body 0 as pelvis: {self.model.body(0).name}")
        if self.torso_body_id is None:
            self.torso_body_id = 1 if self.model.nbody > 1 else 0
            print(f"⚠ Using body {self.torso_body_id} as torso: {self.model.body(self.torso_body_id).name}")

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

    def update_body_anchors(self):
        """
        Update anchor points based on PELVIS și TORSO positions
        Acestea sunt STABILE - nu se mișcă ca picioarele!
        """
        # Get pelvis position (x, y)
        self.pelvis_anchor = self.data.xpos[self.pelvis_body_id][0:2].copy()

        # Get torso position (x, y)
        self.torso_anchor = self.data.xpos[self.torso_body_id][0:2].copy()

        # Target COM = exact la pelvis (centrul de greutate trebuie la bazin!)
        # SAU putem face media între pelvis și torso
        self.target_com = self.pelvis_anchor.copy()  # Forțăm COM la pelvis

    def compute_body_anchor_corrections(self):
        """
        Compute corrections pentru a menține COM la pozițiile anchor rigide
        """
        current_com = self.compute_com()

        # Eroare față de target (pelvis anchor)
        com_error = current_com - self.target_com

        # Corecții FOARTE MARI pentru tracking agresiv
        hip_pitch_correction = -self.pelvis_tracking_gain * com_error[0]
        hip_pitch_correction = np.clip(hip_pitch_correction, -0.20, 0.20)

        ankle_pitch_correction = -self.com_stabilization_gain * com_error[0]
        ankle_pitch_correction = np.clip(ankle_pitch_correction, -0.12, 0.12)

        hip_roll_correction = -self.torso_tracking_gain * com_error[1]
        hip_roll_correction = np.clip(hip_roll_correction, -0.18, 0.18)

        return hip_pitch_correction, ankle_pitch_correction, hip_roll_correction

    def compute_walking_pose(self):
        """
        Compute walking pose cu body anchor tracking
        """
        pose = self.standing_pose.copy()

        # Update anchor points la poziția ACTUALĂ a pelvis/torso
        self.update_body_anchors()

        # Compute corrections - forțăm COM să urmeze pelvis-ul!
        hip_pitch_corr, ankle_pitch_corr, hip_roll_corr = self.compute_body_anchor_corrections()

        phase = self.walking_phase

        # Faze: 0.0-0.5 weight shift, 0.5-0.95 swing, 0.95-1.0 landing

        if phase < 0.5:
            # WEIGHT SHIFT - FOARTE LENT
            shift_progress = phase / 0.5

            if self.current_support == 'left':
                target_shift = 0.06 * shift_progress  # Reduced shift
                pose['dof_left_hip_roll_03'] = -target_shift + hip_roll_corr * 1.5
                pose['dof_right_hip_roll_03'] = target_shift - hip_roll_corr * 0.5
            else:
                target_shift = 0.06 * shift_progress
                pose['dof_left_hip_roll_03'] = target_shift - hip_roll_corr * 0.5
                pose['dof_right_hip_roll_03'] = -target_shift + hip_roll_corr * 1.5

            # Apply STRONG corrections pe ambele picioare
            pose['dof_left_hip_pitch_04'] = 0.02 + hip_pitch_corr
            pose['dof_left_ankle_02'] = -0.02 + ankle_pitch_corr
            pose['dof_right_hip_pitch_04'] = -0.02 + hip_pitch_corr
            pose['dof_right_ankle_02'] = 0.02 + ankle_pitch_corr

        elif phase < 0.95:
            # SWING - MIȘCARE MINIMALĂ
            swing_progress = (phase - 0.5) / 0.45

            # Ultra-minimal forward step
            forward_offset = self.step_forward * swing_progress
            lift = 0.005 * np.sin(swing_progress * np.pi)  # MAX 5mm!!!

            # IK simplificat
            hip_pitch_swing = forward_offset / 0.5
            knee_swing = lift * 2.0

            if self.current_support == 'left':
                # Left supports - COM LOCKED to pelvis!
                pose['dof_left_hip_roll_03'] = -0.06 + hip_roll_corr * 2.0  # DOUBLE correction on support
                pose['dof_right_hip_roll_03'] = 0.06

                # Support leg - RIGID with STRONG corrections
                pose['dof_left_hip_pitch_04'] = 0.02 + hip_pitch_corr * 1.5
                pose['dof_left_knee_04'] = 0.05
                pose['dof_left_ankle_02'] = -0.02 + ankle_pitch_corr * 1.5

                # Swing leg - MINIMAL motion
                pose['dof_right_hip_pitch_04'] = -(0.02 + hip_pitch_swing)
                pose['dof_right_knee_04'] = -(0.05 + knee_swing)
                pose['dof_right_ankle_02'] = 0.02 - hip_pitch_swing * 0.3
            else:
                # Right supports
                pose['dof_left_hip_roll_03'] = 0.06
                pose['dof_right_hip_roll_03'] = -0.06 + hip_roll_corr * 2.0

                # Support leg
                pose['dof_right_hip_pitch_04'] = -0.02 + hip_pitch_corr * 1.5
                pose['dof_right_knee_04'] = -0.05
                pose['dof_right_ankle_02'] = 0.02 + ankle_pitch_corr * 1.5

                # Swing leg
                pose['dof_left_hip_pitch_04'] = 0.02 + hip_pitch_swing
                pose['dof_left_knee_04'] = 0.05 + knee_swing
                pose['dof_left_ankle_02'] = -0.02 - hip_pitch_swing * 0.3

        else:
            # LANDING
            landing_progress = (phase - 0.95) / 0.05
            lateral_shift = 0.06 * (1.0 - landing_progress)

            if self.current_support == 'left':
                pose['dof_left_hip_roll_03'] = -lateral_shift + hip_roll_corr
                pose['dof_right_hip_roll_03'] = lateral_shift - hip_roll_corr
            else:
                pose['dof_left_hip_roll_03'] = lateral_shift - hip_roll_corr
                pose['dof_right_hip_roll_03'] = -lateral_shift + hip_roll_corr

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
            print(f"✓ Step {self.steps_taken}! Pelvis/Torso anchors updated, switching to {self.current_support}")

    def run(self, duration=40.0):
        """Run body anchor walking controller"""
        print(f"\n{'='*75}")
        print(f"BODY ANCHOR WALKING CONTROLLER - robot.mjcf")
        print(f"{'='*75}")
        print(f"Puncte de ancorare pe PELVIS și TORSO (părți rigide, NU picioare!)")
        print(f"COM forțat să urmeze pelvis-ul constant")
        print(f"Step: {self.step_forward*100:.1f}cm forward, 5mm lift MAX")
        print(f"Duration: {self.step_duration}s per step (ULTRA-slow)\n")

        self.set_initial_pose()

        print("Standing for 6 seconds (calibrating body anchors)...")
        start_time = time.time()
        standing_duration = 6.0

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                step_start = time.time()
                elapsed = step_start - start_time

                if elapsed < standing_duration:
                    # Standing - calibrate body anchors
                    self.update_body_anchors()
                    hip_pitch_corr, ankle_pitch_corr, hip_roll_corr = self.compute_body_anchor_corrections()

                    pose = self.standing_pose.copy()
                    pose['dof_left_hip_pitch_04'] += hip_pitch_corr
                    pose['dof_right_hip_pitch_04'] += hip_pitch_corr
                    pose['dof_left_ankle_02'] += ankle_pitch_corr
                    pose['dof_right_ankle_02'] += ankle_pitch_corr
                    pose['dof_left_hip_roll_03'] += hip_roll_corr
                    pose['dof_right_hip_roll_03'] -= hip_roll_corr

                elif self.steps_taken < self.target_steps:
                    # Walking with body anchor tracking
                    pose = self.compute_walking_pose()
                    self.update_phase(self.model.opt.timestep)

                else:
                    # Finished
                    self.update_body_anchors()
                    hip_pitch_corr, ankle_pitch_corr, hip_roll_corr = self.compute_body_anchor_corrections()

                    pose = self.standing_pose.copy()
                    pose['dof_left_hip_pitch_04'] += hip_pitch_corr
                    pose['dof_right_hip_pitch_04'] += hip_pitch_corr
                    pose['dof_left_hip_roll_03'] += hip_roll_corr
                    pose['dof_right_hip_roll_03'] -= hip_roll_corr

                self.send_commands(pose)
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                # Print status
                if int(elapsed * 2) != int((elapsed - self.model.opt.timestep) * 2):
                    base_h = self.data.qpos[2]
                    com = self.compute_com()

                    com_error = np.linalg.norm(com - self.target_com)

                    phase_name = "SHIFT" if self.walking_phase < 0.5 else \
                                 "SWING" if self.walking_phase < 0.95 else "LAND"

                    print(f"T:{elapsed:5.1f}s | Step:{self.steps_taken}/{self.target_steps} | "
                          f"P:{self.walking_phase:.2f}({phase_name:5s}) | {self.current_support:5s} | "
                          f"H:{base_h:.3f}m | COM:({com[0]:6.3f},{com[1]:6.3f}) | "
                          f"Err:{com_error:.4f}m | Pelvis:({self.pelvis_anchor[0]:.3f},{self.pelvis_anchor[1]:.3f})")

                if elapsed > duration:
                    break

                time_left = self.model.opt.timestep - (time.time() - step_start)
                if time_left > 0:
                    time.sleep(time_left)

        total_forward = self.data.qpos[0]
        print(f"\n{'='*75}")
        print(f"✓ Finished after {elapsed:.1f}s")
        print(f"✓ Completed {self.steps_taken}/{self.target_steps} steps")
        print(f"✓ Total forward: {total_forward:.3f}m")
        print(f"{'='*75}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 body_anchor_walking_controller.py <model_path> [duration]")
        sys.exit(1)

    model_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0

    try:
        controller = BodyAnchorWalkingController(model_path)
        controller.run(duration)
    except KeyboardInterrupt:
        print("\n✗ Stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
