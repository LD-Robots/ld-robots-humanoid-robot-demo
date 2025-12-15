#!/usr/bin/env python3
"""
Test LIPM walking controller with MuJoCo simulation.

Complete walking demo integrating:
- Footstep planning
- ZMP preview control
- Inverse kinematics
- MuJoCo physics simulation
"""

import numpy as np
import mujoco
import mujoco.viewer
import sys
from pathlib import Path

# Add parent directory to path to import modules
# sys.path.insert(0, str(Path(__file__).parent.parent))

from lipm_walking_controller.footstep_planner import FootstepPlanner, ZMPTrajectoryGenerator
from lipm_walking_controller.preview_control import DualAxisPreviewController
from lipm_walking_controller.inverse_kinematics import LegIK


class MuJoCoWalkingDemo:
    """Complete walking demo with MuJoCo simulation."""

    def __init__(self, model_path: str, num_steps: int = 4):
        """Initialize simulation and walking controller."""
        print(f"Loading MuJoCo model: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        print(f"✓ Model loaded: {self.model.nbody} bodies, {self.model.njnt} joints, {self.model.nu} actuators")

        # Walking parameters
        self.dt = 0.01  # 100Hz Control Loop
        self.sim_dt = 0.002  # 500Hz Physics Loop
        self.sim_steps_per_control = int(self.dt / self.sim_dt)
        self.com_height = 0.38  # Lower weight for stability
        self.start_delay = 2.0  # Allow 2 seconds to settle/balance before walking

        # Height controller parameters
        self.target_hip_height = 0.44  # Target Z position for hip
        self.kp_height = 50.0  # Proportional gain for height control
        self.kd_height = 10.0  # Derivative gain for height control

        # Joint impedance parameters (PD gains for torque control)
        # Using relatively high gains because we are doing torque control with motors
        self.kp_joint = 6.0   # Proportional gain (Nm/rad approx)
        self.kd_joint = 0.2   # Derivative gain (Nm/s approx)

        # Initialize footstep planner
        print("\n1. Initializing footstep planner...")
        self.footstep_planner = FootstepPlanner(
            step_length=0.00,  # DEBUGGING: No forward motion for now
            step_width=0.16,   # NARROWER: 16cm (was 22cm) to fit IK reach
            step_height=0.03,  # Lift feet by 3cm (appropriate for small robot)
            step_duration=1.0,  # 1 second per step
            double_support_ratio=0.5,  # 50% double support
        )

        # Plan footsteps
        print(f"   Planning {num_steps} steps...")
        self.footsteps = self.footstep_planner.plan_forward_walk(
            num_steps=num_steps,
            start_left=True
        )
        print(f"   ✓ Generated {len(self.footsteps)} footstep waypoints")

        # Generate ZMP trajectory
        print("\n2. Generating ZMP trajectory...")
        self.zmp_generator = ZMPTrajectoryGenerator(dt=self.dt)
        self.time_array, self.zmp_ref_x, self.zmp_ref_y = \
            self.zmp_generator.generate_from_footsteps(self.footsteps)

        total_duration = self.time_array[-1]
        print(f"   ✓ Generated {len(self.time_array)} trajectory points")
        print(f"   ✓ Total walking duration: {total_duration:.2f}s")
        print(f"   DEBUG: ZMP Ref X[:10]: {self.zmp_ref_x[:10]}")
        print(f"   DEBUG: ZMP Ref Y[:10]: {self.zmp_ref_y[:10]}")

        # Initialize preview controller
        print("\n3. Initializing ZMP preview controller...")
        self.preview_controller = DualAxisPreviewController(
            dt=self.dt,
            com_height=self.com_height,
            preview_window=int(1.6 / self.dt),  # 1.6 second preview horizon
        )
        print(f"   ✓ Preview window: {self.preview_controller.controller_x.preview_window} steps")

        # Initialize IK solver
        print("\n4. Initializing inverse kinematics solver...")
        self.leg_ik = LegIK(
            hip_offset_y=0.112,  # From simple_humanoid.xml
            hip_offset_z=0.1,    # Hip vertical offset
            thigh_length=0.212,  # Thigh length
            shin_length=0.240,   # Shin length
            ankle_offset=0.030,  # Ankle offset
        )
        print(f"   ✓ IK configured for robot dimensions")

        # State variables
        self.state_x = np.zeros(3)  # [pos, vel, acc]
        self.state_y = np.zeros(3)
        print(f"DEBUG: Init state_y={self.state_y}")
        self.traj_index = 0
        self.walking_started = False

        # Joint mapping (actuator indices)
        self.joint_indices = {
            'left_hip_yaw': 0,
            'left_hip_roll': 1,
            'left_hip_pitch': 2,
            'left_knee': 3,
            'left_ankle': 4,
            'right_hip_yaw': 5,
            'right_hip_roll': 6,
            'right_hip_pitch': 7,
            'right_knee': 8,
            'right_ankle': 9,
        }

        # Generate foot trajectories
        print("\n5. Generating foot trajectories...")
        self._generate_foot_trajectories()
        print(f"   ✓ Foot trajectories generated")

        # Set initial pose
        print("\n6. Setting initial pose...")
        self._set_initial_pose()
        print(f"   ✓ Robot positioned at starting configuration")

        print("\n✓ All systems initialized!\n")

    def _generate_foot_trajectories(self):
        """Generate smooth foot swing trajectories from footsteps."""
        n_points = len(self.time_array)
        self.left_foot_traj = np.zeros((n_points, 3))
        self.right_foot_traj = np.zeros((n_points, 3))

        # Foot positions in WORLD FRAME (will be converted to hip-relative in compute_control)
        # Z = 0 is ground level where feet should be
        left_y = self.footstep_planner.step_width / 2
        right_y = -self.footstep_planner.step_width / 2
        foot_z = 0.0  # Feet on ground in world frame

        # Track current foot positions
        left_x = 0.0
        right_x = 0.0

        time_idx = 0

        for i, step in enumerate(self.footsteps):
            n_steps = int(step.duration / self.dt)

            # Determine which foot moves
            for j in range(n_steps):
                if time_idx < n_points:
                    # Normalized time in step [0, 1]
                    t_step = j / (n_steps - 1) if n_steps > 1 else 0

                    # Sine wave for foot height (swing phase only)
                    # sin(0) -> 0, sin(pi/2) -> 1, sin(pi) -> 0
                    current_step_height = self.footstep_planner.step_height * np.sin(np.pi * t_step)

                    if step.phase.value == 1:  # LEFT_SUPPORT (right foot swings)
                        # Left foot planted
                        self.left_foot_traj[time_idx] = [left_x, left_y, 0.0]
                        
                        # Right foot swing interpolation
                        rx = right_x + (step.x - right_x) * t_step
                        ry = right_y + (right_y - right_y) * t_step  # y doesn't change for now
                        rz = current_step_height
                        self.right_foot_traj[time_idx] = [rx, ry, rz]
                        
                        if j == n_steps - 1:
                             right_x = step.x

                    elif step.phase.value == 2:  # RIGHT_SUPPORT (left foot swings)
                        # Right foot planted
                        self.right_foot_traj[time_idx] = [right_x, right_y, 0.0]
                        
                        # Left foot swing interpolation
                        lx = left_x + (step.x - left_x) * t_step
                        ly = left_y + (left_y - left_y) * t_step
                        lz = current_step_height
                        self.left_foot_traj[time_idx] = [lx, ly, lz]
                        
                        if j == n_steps - 1:
                            left_x = step.x

                    else:  # DOUBLE_SUPPORT
                        self.left_foot_traj[time_idx] = [left_x, left_y, 0.0]
                        self.right_foot_traj[time_idx] = [right_x, right_y, 0.0]

                    time_idx += 1

    def _set_initial_pose(self):
        """Set robot to initial standing pose."""
        # Set ALL qpos to zero first
        self.data.qpos[:] = 0.0

        # Set position (free joint: x, y, z, qw, qx, qy, qz)
        self.data.qpos[0] = 0.0  # X
        self.data.qpos[1] = 0.0  # Y
        self.data.qpos[2] = 0.45  # Z - Slightly above target (0.44) to settle safely
        self.data.qpos[3] = 1.0  # qw (quaternion)
        self.data.qpos[4:7] = 0.0  # qx, qy, qz

        # Bend knees for stability (same as simple_test)
        try:
            for i in range(self.model.njnt):
                joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
                qpos_idx = self.model.jnt_qposadr[i]

                if joint_name in ['left_knee', 'right_knee']:
                    self.data.qpos[qpos_idx] = 0.3  # Same as simple_test
        except Exception as e:
            print(f"   Warning: Could not set joint angles: {e}")

        # Forward kinematics
        mujoco.mj_forward(self.model, self.data)

        # Save this pose as target for PD control during delay
        self.target_standing_qpos = self.data.qpos.copy()

    def compute_control(self):
        """Compute joint commands for current timestep."""
        if self.traj_index >= len(self.time_array):
            return None

        # Get ZMP reference
        zmp_x_ref = self.zmp_ref_x[self.traj_index]
        zmp_y_ref = self.zmp_ref_y[self.traj_index]

        # Preview window for future ZMP
        preview_len = self.preview_controller.controller_x.preview_window
        end_idx = min(self.traj_index + preview_len, len(self.zmp_ref_x))

        zmp_x_preview = self.zmp_ref_x[self.traj_index+1:end_idx]
        zmp_y_preview = self.zmp_ref_y[self.traj_index+1:end_idx]

        if self.traj_index < 5:
            print(f"DEBUG[step={self.traj_index}] ZMP_ref_y={zmp_y_ref:.4f} ZMP_prev={zmp_y_preview[0] if len(zmp_y_preview)>0 else 0:.4f}")

        u_x, u_y = self.preview_controller.compute_control(
            self.state_x, self.state_y,
            zmp_x_preview, zmp_y_preview,
            zmp_x_ref, zmp_y_ref
        )
        
        if self.traj_index < 5:
            print(f"DEBUG[step={self.traj_index}] u_y={u_y:.4f} state_y={self.state_y}")

        self.state_x, self.state_y = self.preview_controller.update_states(
            self.state_x, self.state_y, u_x, u_y
        )
        # Use ZMP reference directly as COM position (open-loop)
        # Use ZMP reference directly as COM position (open-loop)
        # self.state_x[0] = zmp_x_ref
        # self.state_y[0] = zmp_y_ref

        # Get foot positions from trajectories
        left_foot_pos = self.left_foot_traj[self.traj_index]
        right_foot_pos = self.right_foot_traj[self.traj_index]

        # Compute IK (feet relative to COM position from preview control)
        # Use COM from state, not actual hip position (which drifts due to dynamics)
        com_x = self.state_x[0]
        com_y = self.state_y[0]
        com_z = self.com_height  # Use constant COM height
        com_pos = np.array([com_x, com_y, com_z])

        # Also track actual hip for monitoring
        hip_x = self.data.qpos[0]
        hip_y = self.data.qpos[1]
        hip_z = self.data.qpos[2]
        hip_pos = np.array([hip_x, hip_y, hip_z])

        # HEIGHT CONTROL: Compute correction to maintain hip height
        height_error = self.target_hip_height - hip_z
        height_velocity = self.data.qvel[2]  # Z velocity from free joint

        # PID control: positive correction = extend knees = push hip up
        height_correction = self.kp_height * height_error - self.kd_height * height_velocity
        # Clamp correction to avoid extreme values
        height_correction = np.clip(height_correction, -0.5, 0.5)

        # Transform foot positions to COM-relative coordinates for IK
        left_foot_rel = left_foot_pos - com_pos
        right_foot_rel = right_foot_pos - com_pos

        # DEBUG: Print once every 500 steps
        if self.traj_index % 500 == 0:
            print(f"\n[DEBUG t={self.time_array[self.traj_index]:.2f}s]")
            print(f"  COM state: x={com_x:.3f}, y={com_y:.3f}, z={com_z:.3f}")
            print(f"  Hip pos (actual): x={hip_x:.3f}, y={hip_y:.3f}, z={hip_z:.3f}")
            print(f"  Hip vs COM diff: dx={hip_x-com_x:.3f}, dy={hip_y-com_y:.3f}, dz={hip_z-com_z:.3f}")
            print(f"  Left foot (world): {left_foot_pos}")
            print(f"  Left foot (relative to COM): {left_foot_rel}")
            print(f"  Left foot distance: {np.linalg.norm(left_foot_rel):.3f}m")
            print(f"  Right foot distance: {np.linalg.norm(right_foot_rel):.3f}m")

        left_joints = self.leg_ik.solve(
            left_foot_rel,
            np.array([0, 0, 0]),  # Zero orientation
            is_left=True
        )

        right_joints = self.leg_ik.solve(
            right_foot_rel,
            np.array([0, 0, 0]),
            is_left=False
        )

        if left_joints is None or right_joints is None:
            # IK failed - use current position
            print(f"CRITICAL: IK Failed at index {self.traj_index}!")
            print(f"  State: {self.state_x}, {self.state_y}")
            print(f"  Left target: {left_foot_rel}")
            print(f"  Right target: {right_foot_rel}")
            raise RuntimeError("IK Failed")
            # return self.data.ctrl.copy()

        # Map to actuators with HEIGHT CORRECTION applied to knees
        target_positions = np.zeros(self.model.nu)

        # Left leg (5 DOF: yaw, roll, pitch, knee, ankle)
        target_positions[self.joint_indices['left_hip_yaw']] = left_joints[0]
        target_positions[self.joint_indices['left_hip_roll']] = left_joints[1]
        target_positions[self.joint_indices['left_hip_pitch']] = left_joints[2]
        target_positions[self.joint_indices['left_knee']] = left_joints[3] - height_correction
        target_positions[self.joint_indices['left_ankle']] = left_joints[4]

        # Right leg
        target_positions[self.joint_indices['right_hip_yaw']] = right_joints[0]
        target_positions[self.joint_indices['right_hip_roll']] = right_joints[1]
        target_positions[self.joint_indices['right_hip_pitch']] = right_joints[2]
        target_positions[self.joint_indices['right_knee']] = right_joints[3] - height_correction
        target_positions[self.joint_indices['right_ankle']] = right_joints[4]

        # Convert Target Positions to Tool Control (Torque)
        current_qpos = self.data.qpos[7:]
        current_qvel = self.data.qvel[6:]
        
        ctrl = self._compute_pd_torque(target_positions, current_qpos, current_qvel)

        return ctrl

    def _get_gravity_forces(self):
        """Extract gravity/coriolis forces for actuated joints."""
        # For a floating base robot, the first 6 DOFs in qvel/qfrc_bias are the free joint.
        # The actuated joints usually follow directly.
        
        # Simple assumption for this model:
        # qfrc_bias[0:6] = Free joint
        # qfrc_bias[6:16] = Actuated joints
        return self.data.qfrc_bias[6:]

    def _compute_pd_torque(self, target_qpos, current_qpos, current_qvel):
        """Compute torque = Kp(q_ref - q) - Kd(q_dot) + Gravity."""
        
        # PD Control
        error = target_qpos - current_qpos
        # Clamp error to avoid huge jumps
        # error = np.clip(error, -1.0, 1.0)
        
        pd_torque = self.kp_joint * error - self.kd_joint * current_qvel
        
        # Gravity Compensation
        gravity = self._get_gravity_forces()
        
        # Total torque
        torque = pd_torque + gravity
        
        return torque


    def run(self, headless=False):
        """Run simulation with viewer or headless."""
        print("="*60)
        print("STARTING MUJOCO WALKING SIMULATION")
        print(f"Control DT: {self.dt}s (100Hz) | Sim DT: {self.sim_dt}s (500Hz)")
        print("="*60)
        
        # Standing PD parameters (Soft startup)
        pd_kp = 1.0
        pd_kv = 0.1

        if headless:
            print("Running in HEADLESS mode (no GUI)")
            current_time = 0.0
            step_counter = 0
            last_print_time = 0.0
            ctrl = None  # Zero Order Hold buffer
            
            while self.traj_index < len(self.time_array) or current_time < self.start_delay + 2.0:
                current_time = self.data.time
                
                # Start walking after delay
                if not self.walking_started and current_time > self.start_delay:
                    self.walking_started = True
                    print(f"[t={current_time:.2f}s] 🚶 WALKING STARTED!")

                # Control Loop (100Hz)
                if step_counter % self.sim_steps_per_control == 0:
                    if self.walking_started:
                        try:
                            # Update control (Compute Preview + IK)
                            new_ctrl = self.compute_control()
                            if new_ctrl is not None:
                                ctrl = new_ctrl
                        except RuntimeError:
                             print("Simulating despite IK failure...")
                
                # Apply control (Zero Order Hold or Standing PD)
                if self.walking_started and ctrl is not None:
                    self.data.ctrl[:] = ctrl
                elif not self.walking_started:
                    # Standing Control with Gravity Compensation
                    current_qpos = self.data.qpos[7:]
                    current_qvel = self.data.qvel[6:]
                    target_qpos = self.target_standing_qpos[7:]
                    
                    self.data.ctrl[:] = self._compute_pd_torque(target_qpos, current_qpos, current_qvel)

                # Step simulation
                mujoco.mj_step(self.model, self.data)
                step_counter += 1
                
                # Print progress
                if current_time - last_print_time > 0.5:
                    if self.walking_started and self.traj_index < len(self.time_array):
                        progress = 100 * self.traj_index / len(self.time_array)
                        com_z = self.data.subtree_com[1][2]
                        print(f"[t={current_time:.2f}s] Progress: {progress:5.1f}% | COM z: {com_z:.3f}m | Step: {self.traj_index}")
                    last_print_time = current_time
                    
                # Break if fell
                if self.data.subtree_com[1][2] < 0.2:
                    print(f"\n[t={current_time:.2f}s] ❌ ROBOT FELL! (COM z < 0.2m)")
                    break
            return

        print(f"\nControls:")
        print("  - ESC: Exit simulation")
        print("  - SPACE: Pause/Resume")
        print(f"\nWalking will start after {self.start_delay}s delay...\n")
        
        # Override start_delay to give user time to see standing
        self.start_delay = 4.0
        print(f"DEBUG: Increased start_delay to {self.start_delay}s for standing verification.")

        step_counter = 0
        last_print_time = 0.0
        ctrl = None

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                current_time = self.data.time

                # Start walking after delay
                if not self.walking_started and current_time >= self.start_delay:
                    self.walking_started = True
                    print(f"[t={current_time:.2f}s] 🚶 WALKING STARTED!")
                    print("-" * 60)

                # Control Loop (100Hz)
                if step_counter % self.sim_steps_per_control == 0:
                    if self.walking_started:
                        try:
                            # Update control (Compute Preview + IK)
                            new_ctrl = self.compute_control()
                            if new_ctrl is not None:
                                ctrl = new_ctrl
                            else:
                                # Walking finished
                                if current_time - last_print_time > 1.0:
                                    print(f"\n[t={current_time:.2f}s] ✓ Walking sequence completed!")
                                    last_print_time = current_time
                        except RuntimeError:
                             pass

                # Apply control
                if self.walking_started and ctrl is not None:
                    self.data.ctrl[:] = ctrl
                elif not self.walking_started:
                    # Standing Control with Gravity Compensation
                    if step_counter < 3:
                         print(f"Standing PD + Gravity Comp: Kp={self.kp_joint}, Kv={self.kd_joint}")

                    current_qpos = self.data.qpos[7:]
                    current_qvel = self.data.qvel[6:]
                    target_qpos = self.target_standing_qpos[7:]
                    
                    self.data.ctrl[:] = self._compute_pd_torque(target_qpos, current_qpos, current_qvel)

                # Step simulation
                mujoco.mj_step(self.model, self.data)
                step_counter += 1

                # Sync viewer at 60 FPS
                sync_interval = max(1, int(1.0 / (60 * self.model.opt.timestep)))
                if step_counter % sync_interval == 0:
                    viewer.sync()

                # Print progress
                if self.walking_started and current_time - last_print_time > 1.0:
                    if self.traj_index < len(self.time_array):
                        progress = 100 * self.traj_index / len(self.time_array)
                        com_height = self.data.subtree_com[1][2]
                        print(f"[t={current_time:.2f}s] Progress: {progress:5.1f}% | COM height: {com_height:.3f}m | Step: {self.traj_index}/{len(self.time_array)}")
                        last_print_time = current_time

        print("\n" + "="*60)


def main():
    if len(sys.argv) < 2:
        model_path = "models/simple_humanoid.xml"
    else:
        model_path = sys.argv[1]

    # Number of steps
    num_steps = 4
    if len(sys.argv) >= 3:
        num_steps = int(sys.argv[2])

    print("\n" + "="*60)
    print("LIPM WALKING CONTROLLER - MUJOCO DEMO")
    print("="*60)
    print(f"Model: {model_path}")
    print(f"Steps: {num_steps}")
    print("="*60 + "\n")

    try:
        headless = '--headless' in sys.argv
        demo = MuJoCoWalkingDemo(model_path, num_steps=num_steps)
        demo.run(headless=headless)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
