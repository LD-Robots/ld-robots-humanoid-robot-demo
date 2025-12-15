#!/usr/bin/env python3
"""
ZMP-based balance controller for humanoid standing.

Implements:
- Real-time COM (Center of Mass) computation
- ZMP (Zero Moment Point) stabilization
- Ankle strategy for balance corrections
- PD control on all joints
"""

import numpy as np
import mujoco
import mujoco.viewer
import sys


class ZMPBalanceController:
    """Advanced balance controller using ZMP stabilization."""

    def __init__(self, model_path: str):
        """Initialize MuJoCo simulation and controller."""
        print(f"Loading MuJoCo model: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        print(f"✓ Model loaded: {self.model.nbody} bodies, {self.model.njnt} joints, {self.model.nu} actuators")

        # Control parameters
        self.dt = self.model.opt.timestep
        self.step_count = 0

        # PD gains - ULTRA-HIGH for maximum rigidity (reduced gravity allows this)
        self.kp_hip_knee = 6000.0  # Double previous maximum
        self.kd_hip_knee = 600.0
        self.kp_medium = 4000.0    # Double previous maximum
        self.kd_medium = 400.0
        self.kp_ankle = 1600.0     # Double previous maximum
        self.kd_ankle = 160.0
        self.kp_wrist = 200.0
        self.kd_wrist = 20.0

        # COM/ZMP control gains - ultra-aggressive corrections
        self.hip_correction_gain = 3.0   # Doubled for faster response
        self.ankle_correction_gain = 6.0  # Doubled for faster response

        # Target standing pose - nearly straight for maximum height
        self.target_pose = {
            # Left leg - minimal bend, almost straight
            'dof_left_hip_pitch_04': 0.02,    # Minimal lean
            'dof_left_hip_roll_03': 0.0,
            'dof_left_hip_yaw_03': 0.0,
            'dof_left_knee_04': 0.05,         # Tiny bend ~3 degrees
            'dof_left_ankle_02': -0.02,       # Minimal compensation

            # Right leg - mirror left (opposite conventions)
            'dof_right_hip_pitch_04': -0.02,
            'dof_right_hip_roll_03': 0.0,
            'dof_right_hip_yaw_03': 0.0,
            'dof_right_knee_04': -0.05,       # Tiny bend ~3 degrees
            'dof_right_ankle_02': 0.02,

            # Arms - natural position
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

        # Balance correction offsets (computed by COM controller)
        self.hip_pitch_correction = 0.0
        self.ankle_pitch_correction = 0.0
        self.ankle_roll_correction = 0.0

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

        # Initialize robot pose
        self._set_initial_pose()

        # Get foot body indices for support polygon calculation
        self.left_foot_body_id = None
        self.right_foot_body_id = None

        for i in range(self.model.nbody):
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if 'ankle' in body_name.lower() and 'left' in body_name.lower():
                self.left_foot_body_id = i
            elif 'ankle' in body_name.lower() and 'right' in body_name.lower():
                self.right_foot_body_id = i

        if self.left_foot_body_id and self.right_foot_body_id:
            print(f"✓ Found foot bodies: left={self.left_foot_body_id}, right={self.right_foot_body_id}")

    def _set_initial_pose(self):
        """Set robot to initial standing pose."""
        self.data.qpos[:] = 0.0

        # Floating base (x, y, z, qw, qx, qy, qz)
        self.data.qpos[0] = 0.0
        self.data.qpos[1] = 0.0
        self.data.qpos[2] = 0.85  # Standing height for nearly straight legs
        self.data.qpos[3] = 1.0   # qw
        self.data.qpos[4:7] = 0.0

        # Set target joint angles
        for joint_name, target_angle in self.target_pose.items():
            if joint_name in self.joint_indices:
                joint_idx = self.joint_indices[joint_name]
                qpos_idx = self.model.jnt_qposadr[joint_idx]
                self.data.qpos[qpos_idx] = target_angle

        # Zero velocities
        self.data.qvel[:] = 0.0

        # Forward kinematics
        mujoco.mj_forward(self.model, self.data)

        print("✓ Initial pose set")

    def compute_com(self):
        """Compute center of mass position."""
        # MuJoCo computes COM automatically
        com_pos = self.data.subtree_com[0].copy()  # Root body COM
        return com_pos

    def compute_support_polygon_center(self):
        """Compute center of support polygon (middle point between feet)."""
        if self.left_foot_body_id is None or self.right_foot_body_id is None:
            # Fallback: assume feet at (±0.1, 0, 0) relative to base
            return np.array([0.0, 0.0, 0.0])

        # Get foot positions
        left_foot_pos = self.data.xpos[self.left_foot_body_id].copy()
        right_foot_pos = self.data.xpos[self.right_foot_body_id].copy()

        # Center of support polygon
        center = (left_foot_pos + right_foot_pos) / 2.0
        return center

    def compute_com_error(self):
        """Compute COM error relative to support polygon center."""
        com = self.compute_com()
        support_center = self.compute_support_polygon_center()

        # Error in X and Y (horizontal plane)
        error_x = com[0] - support_center[0]
        error_y = com[1] - support_center[1]

        return error_x, error_y

    def update_balance_corrections(self):
        """Update hip and ankle corrections based on COM error."""
        error_x, error_y = self.compute_com_error()

        # Hip strategy: adjust hip pitch to shift COM (stronger, for large errors)
        self.hip_pitch_correction = -self.hip_correction_gain * error_x
        max_hip_correction = 0.4  # ~23 degrees
        self.hip_pitch_correction = np.clip(self.hip_pitch_correction,
                                            -max_hip_correction, max_hip_correction)

        # Ankle strategy: fine adjustments (for small errors)
        self.ankle_pitch_correction = -self.ankle_correction_gain * error_x
        max_ankle_correction = 0.3  # ~17 degrees
        self.ankle_pitch_correction = np.clip(self.ankle_pitch_correction,
                                              -max_ankle_correction, max_ankle_correction)

        self.ankle_roll_correction = -self.ankle_correction_gain * error_y
        self.ankle_roll_correction = np.clip(self.ankle_roll_correction,
                                             -max_ankle_correction, max_ankle_correction)

    def compute_pd_control(self):
        """Compute PD control with COM-based balance corrections."""
        # Update hip and ankle corrections based on COM
        self.update_balance_corrections()

        # Zero all controls
        self.data.ctrl[:] = 0.0

        # Apply PD control to each joint
        for joint_name, base_target in self.target_pose.items():
            if joint_name not in self.joint_indices:
                continue

            joint_idx = self.joint_indices[joint_name]
            qpos_idx = self.model.jnt_qposadr[joint_idx]
            qvel_idx = self.model.jnt_dofadr[joint_idx]

            current_angle = self.data.qpos[qpos_idx]
            current_velocity = self.data.qvel[qvel_idx]

            # Modify target based on COM error (hip and ankle strategy)
            target_angle = base_target

            # Hip strategy - adjust hip pitch for large COM errors
            if 'hip_pitch' in joint_name:
                target_angle += self.hip_pitch_correction

            # Ankle strategy - fine adjustments for small COM errors
            if 'ankle' in joint_name:
                if 'pitch' in joint_name or '02' in joint_name:  # Ankle pitch
                    target_angle += self.ankle_pitch_correction

            # POSITION control - send target angle directly to MuJoCo position actuator
            # MuJoCo handles PD control internally with kp gains defined in MJCF
            actuator_name = joint_name + '_ctrl'
            if actuator_name in self.actuator_indices:
                actuator_idx = self.actuator_indices[actuator_name]
                self.data.ctrl[actuator_idx] = target_angle  # Position command, not torque!

    def run(self, duration: float = 30.0):
        """Run simulation with ZMP balance controller."""
        print(f"\nStarting ZMP balance controller for {duration} seconds...")
        print("Controls:")
        print("  - Spacebar: Pause/Resume")
        print("  - Mouse drag: Rotate view")
        print("  - Scroll: Zoom")
        print("  - Ctrl+Q or ESC: Quit")
        print("\nPress Ctrl+C to stop\n")

        self.start_time = self.data.time

        def controller_callback(model, data):
            """Control callback."""
            self.compute_pd_control()
            self.step_count += 1

            # Print debug info
            if self.step_count % 500 == 0:
                com = self.compute_com()
                support_center = self.compute_support_polygon_center()
                error_x, error_y = self.compute_com_error()
                base_z = data.qpos[2]

                print(f"Time: {data.time:.2f}s | Base: {base_z:.3f}m | "
                      f"COM: ({com[0]:.3f}, {com[1]:.3f}) | "
                      f"Error: ({error_x:.3f}, {error_y:.3f}) | "
                      f"Ankle corr: {self.ankle_pitch_correction:.3f}rad")

        # Launch viewer
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            viewer.cam.distance = 3.0
            viewer.cam.azimuth = 45
            viewer.cam.elevation = -20

            # Simulation loop
            while viewer.is_running() and (self.data.time - self.start_time) < duration:
                controller_callback(self.model, self.data)
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

        print(f"\n✓ Simulation completed: {self.data.time:.2f}s, {self.step_count} steps")
        print(f"  Final base height: {self.data.qpos[2]:.3f}m")


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage: python3 zmp_balance_controller.py <model_path> [duration]")
        print("\nExample:")
        print("  python3 zmp_balance_controller.py models/robot.mjcf 60")
        sys.exit(1)

    model_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) >= 3 else 60.0

    print("\n" + "="*60)
    print("ZMP BALANCE CONTROLLER - STANDING STABILITY")
    print("="*60)
    print(f"Model: {model_path}")
    print(f"Duration: {duration}s")
    print("="*60 + "\n")

    try:
        controller = ZMPBalanceController(model_path)
        controller.run(duration=duration)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
