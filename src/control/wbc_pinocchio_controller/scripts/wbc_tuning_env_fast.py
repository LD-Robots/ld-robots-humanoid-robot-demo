#!/usr/bin/env python3
"""
Fast lightweight Gymnasium environment for WBC parameter tuning.
Uses MuJoCo directly without ROS2 for efficient parallel training.
"""

import gymnasium as gym
import numpy as np
from typing import Optional, Dict, Tuple
import yaml
from pathlib import Path
import mujoco
import mujoco.viewer


class WbcTuningEnvFast(gym.Env):
    """Fast RL environment using MuJoCo directly (no ROS2)."""

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 100}

    def __init__(
        self,
        config_path: str = None,
        max_steps: int = 1000,
        render_mode: Optional[str] = None,
    ):
        super().__init__()

        self.max_steps = max_steps
        self.render_mode = render_mode
        self.step_count = 0

        # Paths
        self.pkg_path = Path(__file__).parent.parent
        self.config_path = config_path or str(self.pkg_path / "config" / "wbc_controller.yaml")

        # Load robot model
        robot_xml_path = self.pkg_path.parent.parent / "robot_description" / "humanoid_description" / "urdf" / "robot.xml"
        self.model = mujoco.MjModel.from_xml_path(str(robot_xml_path))
        self.data = mujoco.MjData(self.model)

        # Viewer for rendering
        self.viewer = None

        # Joint mapping (MuJoCo joint names)
        self.joint_names = [
            'left_hip_yaw_joint', 'left_hip_roll_joint', 'left_hip_pitch_joint',
            'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            'right_hip_yaw_joint', 'right_hip_roll_joint', 'right_hip_pitch_joint',
            'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
        ]

        # Get joint IDs
        self.joint_ids = []
        for name in self.joint_names:
            try:
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                self.joint_ids.append(joint_id)
            except:
                print(f"Warning: Joint {name} not found in model")

        # WBC parameters (will be set by action)
        self.wbc_params = {}

        # State tracking
        self.fell = False
        self.total_distance = 0.0
        self.initial_com_x = 0.0
        self.last_com_x = 0.0
        self.steps_taken = 0

        # Action space: ALL tunable parameters (39 params)
        self.action_space = gym.spaces.Box(
            low=np.array([
                # Timing
                0.3, 0.5, 0.25,
                # Thresholds
                0.005, 0.05, 0.01, 0.03, 0.02, 0.0001, 0.7,
                # Step parameters
                0.015, 0.01, 0.08,
                # Knee angles
                0.05, 0.2,
                # Hip pitch
                -0.05, 0.01,
                # Ankle pitch
                -0.05, -0.05,
                # Hip roll
                0.0, -0.05,
                # Hip yaw
                -0.05, -0.05,
                # Balance gains - pitch
                0.5, 0.1, 0.1, 0.01,
                # Balance gains - roll
                1.0, 0.1, 0.3, 0.01,
                # Limits
                0.15, 0.25, 0.01, 0.01,
                # Other
                0.001, 0.01, 0.1, -0.08,
            ], dtype=np.float32),
            high=np.array([
                # Timing
                1.5, 3.0, 0.8,
                # Thresholds
                0.03, 0.4, 0.05, 0.15, 0.1, 0.01, 0.95,
                # Step parameters
                0.08, 0.04, 0.15,
                # Knee angles
                0.35, 0.6,
                # Hip pitch
                0.25, 0.35,
                # Ankle pitch
                0.25, 0.25,
                # Hip roll
                0.15, 0.10,
                # Hip yaw
                0.10, 0.10,
                # Balance gains - pitch
                5.0, 2.0, 2.0, 0.5,
                # Balance gains - roll
                5.0, 1.5, 2.0, 0.3,
                # Limits
                0.35, 0.6, 0.05, 0.05,
                # Other
                0.015, 0.08, 0.5, 0.02,
            ], dtype=np.float32),
            dtype=np.float32
        )

        # Observation space: robot state
        # [12 joint pos, 12 joint vel, 3 base orientation (euler), 3 base ang vel, 3 COM pos, 3 COM vel] = 36
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(36,),
            dtype=np.float32
        )

    def _parse_params(self, params: np.ndarray) -> Dict:
        """Parse action array into WBC parameter dictionary."""
        return {
            # Timing
            'hold_enter_duration': float(params[0]),
            'stabilize_duration': float(params[1]),
            'step_duration': float(params[2]),
            # Thresholds
            'hold_com_threshold': float(params[3]),
            'hold_vel_threshold': float(params[4]),
            'hold_release_com_threshold': float(params[5]),
            'hold_release_vel_threshold': float(params[6]),
            'phase_error_threshold': float(params[7]),
            'phase_hold_max': float(params[8]),
            'phase_hold_progress_gate': float(params[9]),
            # Step parameters
            'step_length': float(params[10]),
            'step_height': float(params[11]),
            'stance_width': float(params[12]),
            # Knee angles
            'support_knee_bend': float(params[13]),
            'swing_knee_bend': float(params[14]),
            # Hip pitch
            'support_hip_pitch': float(params[15]),
            'swing_hip_pitch': float(params[16]),
            # Ankle pitch
            'support_ankle_pitch': float(params[17]),
            'swing_ankle_pitch': float(params[18]),
            # Hip roll
            'hip_roll_shift': float(params[19]),
            'swing_hip_roll_out': float(params[20]),
            # Hip yaw
            'support_hip_yaw': float(params[21]),
            'swing_hip_yaw': float(params[22]),
            # Balance gains - pitch
            'balance_kp_pitch': float(params[23]),
            'balance_kd_pitch': float(params[24]),
            'imu_kp_pitch': float(params[25]),
            'imu_kd_pitch': float(params[26]),
            # Balance gains - roll
            'balance_kp_roll': float(params[27]),
            'balance_kd_roll': float(params[28]),
            'imu_kp_roll': float(params[29]),
            'imu_kd_roll': float(params[30]),
            # Limits
            'ankle_pitch_limit': float(params[31]),
            'hip_pitch_limit': float(params[32]),
            'ankle_roll_limit': float(params[33]),
            'hip_roll_limit': float(params[34]),
            # Other
            'com_deadzone': float(params[35]),
            'prediction_time': float(params[36]),
            'filter_alpha': float(params[37]),
            'support_center_x_offset': float(params[38]),
        }

    def _compute_joint_targets(self, phase_progress: float) -> np.ndarray:
        """
        Compute joint position targets based on WBC parameters and gait phase.
        Simplified walking controller that cycles between left/right stance.
        """
        p = self.wbc_params
        targets = np.zeros(12)

        # Determine which leg is stance/swing based on time
        cycle_time = 2.0 * p['step_duration']  # Full walking cycle
        phase = (self.step_count * self.model.opt.timestep) % cycle_time

        if phase < p['step_duration']:
            # Left stance, right swing
            stance_sign = 1
        else:
            # Right stance, left swing
            stance_sign = -1

        swing_progress = (phase % p['step_duration']) / p['step_duration']
        swing_factor = np.sin(swing_progress * np.pi)  # Smooth 0→1→0

        # Left leg: hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll
        if stance_sign > 0:  # Left is stance
            targets[0] = p['support_hip_yaw']
            targets[1] = -p['hip_roll_shift']
            targets[2] = -p['support_hip_pitch']
            targets[3] = p['support_knee_bend']
            targets[4] = p['support_ankle_pitch']
            targets[5] = 0.0
        else:  # Left is swing
            targets[0] = p['swing_hip_yaw']
            targets[1] = p['swing_hip_roll_out']
            targets[2] = -(p['support_hip_pitch'] + p['swing_hip_pitch'] * swing_factor)
            targets[3] = p['support_knee_bend'] + p['swing_knee_bend'] * swing_factor
            targets[4] = p['swing_ankle_pitch'] * swing_factor
            targets[5] = 0.0

        # Right leg
        if stance_sign < 0:  # Right is stance
            targets[6] = p['support_hip_yaw']
            targets[7] = p['hip_roll_shift']
            targets[8] = -p['support_hip_pitch']
            targets[9] = p['support_knee_bend']
            targets[10] = p['support_ankle_pitch']
            targets[11] = 0.0
        else:  # Right is swing
            targets[6] = p['swing_hip_yaw']
            targets[7] = -p['swing_hip_roll_out']
            targets[8] = -(p['support_hip_pitch'] + p['swing_hip_pitch'] * swing_factor)
            targets[9] = p['support_knee_bend'] + p['swing_knee_bend'] * swing_factor
            targets[10] = p['swing_ankle_pitch'] * swing_factor
            targets[11] = 0.0

        return targets

    def _get_obs(self) -> np.ndarray:
        """Get current observation."""
        obs = np.zeros(36, dtype=np.float32)

        # Joint positions (12)
        qpos_start = self.model.nq - len(self.joint_ids)
        obs[0:12] = self.data.qpos[qpos_start:qpos_start+12]

        # Joint velocities (12)
        qvel_start = self.model.nv - len(self.joint_ids)
        obs[12:24] = self.data.qvel[qvel_start:qvel_start+12]

        # Base orientation (euler from quaternion) (3)
        quat = self.data.qpos[3:7]  # Base quaternion
        euler = self._quat_to_euler(quat)
        obs[24:27] = euler

        # Base angular velocity (3)
        obs[27:30] = self.data.qvel[3:6]

        # COM position (3)
        com_pos = self.data.subtree_com[0]
        obs[30:33] = com_pos

        # COM velocity (3)
        mujoco.mj_comVel(self.model, self.data)
        obs[33:36] = self.data.cvel[0, 3:6]  # Linear velocity

        return obs

    def _quat_to_euler(self, quat):
        """Convert quaternion [w,x,y,z] to euler angles [roll, pitch, yaw]."""
        w, x, y, z = quat
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        pitch = np.arcsin(2*(w*y - z*x))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        return np.array([roll, pitch, yaw])

    def _check_termination(self) -> Tuple[bool, str]:
        """Check if episode should terminate."""
        # Check if robot fell (pelvis height too low or tilted too much)
        pelvis_height = self.data.qpos[2]  # Z position
        if pelvis_height < 0.4:  # Below 40cm
            return True, "fell_low"

        # Check orientation
        quat = self.data.qpos[3:7]
        euler = self._quat_to_euler(quat)
        if abs(euler[0]) > 0.5 or abs(euler[1]) > 0.5:  # Roll or pitch > ~30 deg
            return True, "fell_tilt"

        # Check if went backward
        com_pos = self.data.subtree_com[0]
        if com_pos[0] < self.initial_com_x - 0.1:  # Moved 10cm backward
            return True, "backward"

        return False, ""

    def _compute_reward(self) -> float:
        """Compute reward for current state."""
        reward = 0.0

        # Main reward: forward progress
        com_pos = self.data.subtree_com[0]
        delta_x = com_pos[0] - self.last_com_x
        self.last_com_x = com_pos[0]
        self.total_distance = com_pos[0] - self.initial_com_x

        # Reward for moving forward (most important!)
        reward += delta_x * 100.0  # 1cm = 1.0 reward

        # Penalty for lateral deviation
        lateral_dev = abs(com_pos[1])
        reward -= lateral_dev * 10.0

        # Reward for staying upright
        quat = self.data.qpos[3:7]
        euler = self._quat_to_euler(quat)
        tilt_penalty = (euler[0]**2 + euler[1]**2) * 5.0
        reward -= tilt_penalty

        # Small penalty for energy use (smooth motion)
        ctrl_cost = np.sum(self.data.ctrl**2) * 0.001
        reward -= ctrl_cost

        # Bonus for survival
        reward += 0.1

        return float(reward)

    def reset(self, seed=None, options=None):
        """Reset environment to initial state."""
        super().reset(seed=seed)

        # Reset MuJoCo
        mujoco.mj_resetData(self.model, self.data)

        # Set initial standing pose
        qpos_start = self.model.nq - 12
        self.data.qpos[qpos_start:qpos_start+12] = [
            0.0, 0.0, -0.1,  # Left leg: yaw, roll, pitch
            0.15, 0.05, 0.0,  # knee, ankle_pitch, ankle_roll
            0.0, 0.0, -0.1,  # Right leg
            0.15, 0.05, 0.0,
        ]

        # Reset pelvis height
        self.data.qpos[2] = 0.65  # Standing height

        mujoco.mj_forward(self.model, self.data)

        # Initialize tracking
        self.step_count = 0
        self.fell = False
        com_pos = self.data.subtree_com[0]
        self.initial_com_x = com_pos[0]
        self.last_com_x = com_pos[0]
        self.total_distance = 0.0
        self.steps_taken = 0

        obs = self._get_obs()
        info = {}

        return obs, info

    def step(self, action):
        """Execute one step with given action (WBC parameters)."""
        # Parse parameters from action
        self.wbc_params = self._parse_params(action)

        # Compute joint targets based on current phase
        phase_progress = (self.step_count * self.model.opt.timestep) % self.wbc_params['step_duration']
        targets = self._compute_joint_targets(phase_progress)

        # Apply PD control to reach targets
        kp = 200.0
        kd = 20.0

        qpos_start = self.model.nq - 12
        qvel_start = self.model.nv - 12

        for i in range(12):
            pos_error = targets[i] - self.data.qpos[qpos_start + i]
            vel_error = -self.data.qvel[qvel_start + i]
            self.data.ctrl[i] = kp * pos_error + kd * vel_error

        # Step simulation
        mujoco.mj_step(self.model, self.data)
        self.step_count += 1

        # Get observation
        obs = self._get_obs()

        # Check termination
        terminated, reason = self._check_termination()
        truncated = self.step_count >= self.max_steps

        # Compute reward
        reward = self._compute_reward()

        # Penalty for falling
        if terminated:
            reward -= 50.0
            self.fell = True

        # Bonus for completing episode
        if truncated and not terminated:
            reward += 100.0

        # Info
        info = {
            'distance': self.total_distance,
            'steps_taken': self.steps_taken,
            'fell': self.fell,
            'reason': reason if terminated else 'success' if truncated else 'ongoing',
        }

        # Render if needed
        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, info

    def render(self):
        """Render the environment."""
        if self.render_mode == "human":
            if self.viewer is None:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.sync()

    def close(self):
        """Clean up resources."""
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
