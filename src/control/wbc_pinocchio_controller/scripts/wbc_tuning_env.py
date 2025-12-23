#!/usr/bin/env python3
"""
Gymnasium environment for PPO-based WBC parameter tuning.
Wraps MuJoCo simulation and WBC controller for RL training.
"""

import gymnasium as gym
import numpy as np
from typing import Dict, Tuple, Optional
import subprocess
import time
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PoseStamped


class WbcTuningEnv(gym.Env):
    """RL environment for tuning WBC walking parameters."""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(
        self,
        config_path: str = None,
        max_steps: int = 1000,  # 10 seconds @ 100Hz
        render_mode: Optional[str] = None,
    ):
        super().__init__()

        self.max_steps = max_steps
        self.render_mode = render_mode
        self.step_count = 0

        # ROS2 setup
        if not rclpy.ok():
            rclpy.init()

        # Paths
        self.pkg_path = Path(__file__).parent.parent
        self.config_path = config_path or str(self.pkg_path / "config" / "wbc_controller.yaml")
        self.backup_config = None

        # State tracking
        self.joint_state = None
        self.imu_msg = None
        self.base_pose = None
        self.fell = False
        self.steps_taken = 0
        self.total_distance = 0.0
        self.last_com_x = 0.0

        # Load current config values as baseline
        self.baseline_params = self._load_baseline_config()

        # Action space: relative adjustments to baseline (±30% variation)
        # Action represents MULTIPLIER: 0.7 to 1.3 of baseline values
        self.action_space = gym.spaces.Box(
            low=np.full(39, 0.7, dtype=np.float32),  # -30% of baseline
            high=np.full(39, 1.3, dtype=np.float32),  # +30% of baseline
            dtype=np.float32
        )

        # OLD absolute action space (commented for reference):
        """
        self.action_space_absolute = gym.spaces.Box(
            low=np.array([
                # Timing
                0.3,    # 0:  hold_enter_duration
                0.5,    # 1:  stabilize_duration
                0.25,   # 2:  step_duration
                # Thresholds
                0.005,  # 3:  hold_com_threshold
                0.05,   # 4:  hold_vel_threshold
                0.01,   # 5:  hold_release_com_threshold
                0.03,   # 6:  hold_release_vel_threshold
                0.02,   # 7:  phase_error_threshold
                0.0001, # 8:  phase_hold_max
                0.7,    # 9:  phase_hold_progress_gate
                # Step parameters
                0.015,  # 10: step_length
                0.01,   # 11: step_height
                0.08,   # 12: stance_width
                # Knee angles
                0.05,   # 13: support_knee_bend
                0.2,    # 14: swing_knee_bend
                # Hip pitch
                -0.05,  # 15: support_hip_pitch
                0.01,   # 16: swing_hip_pitch
                # Ankle pitch
                -0.05,  # 17: support_ankle_pitch
                -0.05,  # 18: swing_ankle_pitch
                # Hip roll
                0.0,    # 19: hip_roll_shift
                -0.05,  # 20: swing_hip_roll_out
                # Hip yaw
                -0.05,  # 21: support_hip_yaw
                -0.05,  # 22: swing_hip_yaw
                # Balance gains - pitch
                0.5,    # 23: balance_kp_pitch
                0.1,    # 24: balance_kd_pitch
                0.1,    # 25: imu_kp_pitch
                0.01,   # 26: imu_kd_pitch
                # Balance gains - roll
                1.0,    # 27: balance_kp_roll
                0.1,    # 28: balance_kd_roll
                0.3,    # 29: imu_kp_roll
                0.01,   # 30: imu_kd_roll
                # Limits
                0.15,   # 31: ankle_pitch_limit
                0.25,   # 32: hip_pitch_limit
                0.01,   # 33: ankle_roll_limit
                0.01,   # 34: hip_roll_limit
                # Other
                0.001,  # 35: com_deadzone
                0.01,   # 36: prediction_time
                0.1,    # 37: filter_alpha
                -0.08,  # 38: support_center_x_offset
            ], dtype=np.float32),
            high=np.array([
                # Timing
                1.5,    # 0:  hold_enter_duration
                3.0,    # 1:  stabilize_duration
                0.8,    # 2:  step_duration
                # Thresholds
                0.03,   # 3:  hold_com_threshold
                0.4,    # 4:  hold_vel_threshold
                0.05,   # 5:  hold_release_com_threshold
                0.15,   # 6:  hold_release_vel_threshold
                0.1,    # 7:  phase_error_threshold
                0.01,   # 8:  phase_hold_max
                0.95,   # 9:  phase_hold_progress_gate
                # Step parameters
                0.08,   # 10: step_length
                0.04,   # 11: step_height
                0.15,   # 12: stance_width
                # Knee angles
                0.35,   # 13: support_knee_bend
                0.6,    # 14: swing_knee_bend
                # Hip pitch
                0.25,   # 15: support_hip_pitch
                0.35,   # 16: swing_hip_pitch
                # Ankle pitch
                0.25,   # 17: support_ankle_pitch
                0.25,   # 18: swing_ankle_pitch
                # Hip roll
                0.15,   # 19: hip_roll_shift
                0.10,   # 20: swing_hip_roll_out
                # Hip yaw
                0.10,   # 21: support_hip_yaw
                0.10,   # 22: swing_hip_yaw
                # Balance gains - pitch
                5.0,    # 23: balance_kp_pitch
                2.0,    # 24: balance_kd_pitch
                2.0,    # 25: imu_kp_pitch
                0.5,    # 26: imu_kd_pitch
                # Balance gains - roll
                5.0,    # 27: balance_kp_roll
                1.5,    # 28: balance_kd_roll
                2.0,    # 29: imu_kp_roll
                0.3,    # 30: imu_kd_roll
                # Limits
                0.35,   # 31: ankle_pitch_limit
                0.6,    # 32: hip_pitch_limit
                0.05,   # 33: ankle_roll_limit
                0.05,   # 34: hip_roll_limit
                # Other
                0.015,  # 35: com_deadzone
                0.08,   # 36: prediction_time
                0.5,    # 37: filter_alpha
                0.02,   # 38: support_center_x_offset
            ], dtype=np.float32),
            dtype=np.float32
        )
        """

        # Observation space: robot state (45 dims)
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(45,),
            dtype=np.float32
        )

        # Simulation process
        self.sim_process = None

    def _load_baseline_config(self) -> np.ndarray:
        """Load current config values as baseline for relative adjustments."""
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)

        wbc_params = config['wbc_controller']['ros__parameters']

        # Extract all 39 tunable parameters in order
        baseline = np.array([
            # Timing
            wbc_params.get('hold_enter_duration', 0.8),
            wbc_params.get('stabilize_duration', 2.0),
            wbc_params.get('step_duration', 0.3),
            # Thresholds
            wbc_params.get('hold_com_threshold', 0.015),
            wbc_params.get('hold_vel_threshold', 0.2),
            wbc_params.get('hold_release_com_threshold', 0.02),
            wbc_params.get('hold_release_vel_threshold', 0.08),
            wbc_params.get('phase_error_threshold', 0.06),
            wbc_params.get('phase_hold_max', 0.001),
            wbc_params.get('phase_hold_progress_gate', 0.9),
            # Step parameters
            wbc_params.get('step_length', 0.025),
            wbc_params.get('step_height', 0.015),
            wbc_params.get('stance_width', 0.1),
            # Knee angles
            wbc_params.get('support_knee_bend', 0.2),
            wbc_params.get('swing_knee_bend', 0.4),
            # Hip pitch
            wbc_params.get('support_hip_pitch', 0.35),
            wbc_params.get('swing_hip_pitch', 0.47),
            # Ankle pitch
            wbc_params.get('support_ankle_pitch', 0.22),
            wbc_params.get('swing_ankle_pitch', 0.4),
            # Hip roll
            wbc_params.get('hip_roll_shift', 0.0),
            wbc_params.get('swing_hip_roll_out', 0.0),
            # Hip yaw
            wbc_params.get('support_hip_yaw', 0.0),
            wbc_params.get('swing_hip_yaw', 0.0),
            # Balance gains - pitch
            wbc_params.get('balance_kp_pitch', 2.5),
            wbc_params.get('balance_kd_pitch', 0.8),
            wbc_params.get('imu_kp_pitch', 0.9),
            wbc_params.get('imu_kd_pitch', 0.1),
            # Balance gains - roll
            wbc_params.get('balance_kp_roll', 3.0),
            wbc_params.get('balance_kd_roll', 0.4),
            wbc_params.get('imu_kp_roll', 0.8),
            wbc_params.get('imu_kd_roll', 0.1),
            # Limits
            wbc_params.get('ankle_pitch_limit', 0.22),
            wbc_params.get('hip_pitch_limit', 0.45),
            wbc_params.get('ankle_roll_limit', 0.02),
            wbc_params.get('hip_roll_limit', 0.02),
            # Other
            wbc_params.get('com_deadzone', 0.005),
            wbc_params.get('prediction_time', 0.04),
            wbc_params.get('filter_alpha', 0.25),
            wbc_params.get('support_center_x_offset', -0.001),
        ], dtype=np.float32)

        print(f"✓ Loaded baseline config with {len(baseline)} parameters")
        return baseline

    def _apply_config(self, params: np.ndarray):
        """
        Apply RL action to WBC config file.
        Params are MULTIPLIERS (0.7-1.3) applied to baseline values.
        """
        # Compute actual values: baseline * multiplier
        actual_values = self.baseline_params * params

        # Read current config
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Backup on first call
        if self.backup_config is None:
            self.backup_config = config.copy()

        # Update ALL tunable parameters (39 params)
        wbc_params = config['wbc_controller']['ros__parameters']

        # Timing
        wbc_params['hold_enter_duration'] = float(actual_values[0])
        wbc_params['stabilize_duration'] = float(actual_values[1])
        wbc_params['step_duration'] = float(actual_values[2])

        # Thresholds
        wbc_params['hold_com_threshold'] = float(actual_values[3])
        wbc_params['hold_vel_threshold'] = float(actual_values[4])
        wbc_params['hold_release_com_threshold'] = float(actual_values[5])
        wbc_params['hold_release_vel_threshold'] = float(actual_values[6])
        wbc_params['phase_error_threshold'] = float(actual_values[7])
        wbc_params['phase_hold_max'] = float(actual_values[8])
        wbc_params['phase_hold_progress_gate'] = float(actual_values[9])

        # Step parameters
        wbc_params['step_length'] = float(actual_values[10])
        wbc_params['step_height'] = float(actual_values[11])
        wbc_params['stance_width'] = float(actual_values[12])

        # Knee angles
        wbc_params['support_knee_bend'] = float(actual_values[13])
        wbc_params['swing_knee_bend'] = float(actual_values[14])

        # Hip pitch
        wbc_params['support_hip_pitch'] = float(actual_values[15])
        wbc_params['swing_hip_pitch'] = float(actual_values[16])

        # Ankle pitch
        wbc_params['support_ankle_pitch'] = float(actual_values[17])
        wbc_params['swing_ankle_pitch'] = float(actual_values[18])

        # Hip roll
        wbc_params['hip_roll_shift'] = float(actual_values[19])
        wbc_params['swing_hip_roll_out'] = float(actual_values[20])

        # Hip yaw
        wbc_params['support_hip_yaw'] = float(actual_values[21])
        wbc_params['swing_hip_yaw'] = float(actual_values[22])

        # Balance gains - pitch
        wbc_params['balance_kp_pitch'] = float(actual_values[23])
        wbc_params['balance_kd_pitch'] = float(actual_values[24])
        wbc_params['imu_kp_pitch'] = float(actual_values[25])
        wbc_params['imu_kd_pitch'] = float(actual_values[26])

        # Balance gains - roll
        wbc_params['balance_kp_roll'] = float(actual_values[27])
        wbc_params['balance_kd_roll'] = float(actual_values[28])
        wbc_params['imu_kp_roll'] = float(actual_values[29])
        wbc_params['imu_kd_roll'] = float(actual_values[30])

        # Limits
        wbc_params['ankle_pitch_limit'] = float(actual_values[31])
        wbc_params['hip_pitch_limit'] = float(actual_values[32])
        wbc_params['ankle_roll_limit'] = float(actual_values[33])
        wbc_params['hip_roll_limit'] = float(actual_values[34])

        # Other
        wbc_params['com_deadzone'] = float(actual_values[35])
        wbc_params['prediction_time'] = float(actual_values[36])
        wbc_params['filter_alpha'] = float(actual_values[37])
        wbc_params['support_center_x_offset'] = float(actual_values[38])

        # Write updated config
        with open(self.config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    def _start_simulation(self):
        """Launch ROS2 simulation with current config."""
        # Kill any existing simulation
        self._stop_simulation()

        # Launch MuJoCo + WBC
        launch_cmd = [
            'ros2', 'launch',
            'humanoid_mujoco', 'mujoco_with_wbc.launch.py'
        ]

        self.sim_process = subprocess.Popen(
            launch_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=lambda: None
        )

        # Wait for nodes to start
        time.sleep(3.0)

        # Create ROS2 subscribers
        self.node = rclpy.create_node('wbc_rl_env')
        self.joint_sub = self.node.create_subscription(
            JointState, '/joint_states', self._joint_callback, 10
        )
        self.imu_sub = self.node.create_subscription(
            Imu, '/imu/data', self._imu_callback, 10
        )
        self.pose_sub = self.node.create_subscription(
            PoseStamped, '/pelvis/pose', self._pose_callback, 10
        )

    def _stop_simulation(self):
        """Stop ROS2 simulation."""
        if self.sim_process is not None:
            self.sim_process.terminate()
            self.sim_process.wait(timeout=5)
            self.sim_process = None

        if hasattr(self, 'node'):
            self.node.destroy_node()

    def _joint_callback(self, msg: JointState):
        self.joint_state = msg

    def _imu_callback(self, msg: Imu):
        self.imu_msg = msg

    def _pose_callback(self, msg: PoseStamped):
        self.base_pose = msg

    def _get_observation(self) -> np.ndarray:
        """Build observation vector from robot state."""
        obs = np.zeros(45, dtype=np.float32)

        if self.base_pose is not None:
            # Base position (3)
            obs[0] = self.base_pose.pose.position.x
            obs[1] = self.base_pose.pose.position.y
            obs[2] = self.base_pose.pose.position.z

            # Base orientation quaternion (4)
            obs[3] = self.base_pose.pose.orientation.w
            obs[4] = self.base_pose.pose.orientation.x
            obs[5] = self.base_pose.pose.orientation.y
            obs[6] = self.base_pose.pose.orientation.z

        if self.joint_state is not None and len(self.joint_state.position) >= 12:
            # Joint positions (12)
            obs[7:19] = self.joint_state.position[:12]

            # Joint velocities (12)
            if len(self.joint_state.velocity) >= 12:
                obs[19:31] = self.joint_state.velocity[:12]

        if self.imu_msg is not None:
            # IMU linear acceleration (3)
            obs[31] = self.imu_msg.linear_acceleration.x
            obs[32] = self.imu_msg.linear_acceleration.y
            obs[33] = self.imu_msg.linear_acceleration.z

            # IMU angular velocity (3)
            obs[34] = self.imu_msg.angular_velocity.x
            obs[35] = self.imu_msg.angular_velocity.y
            obs[36] = self.imu_msg.angular_velocity.z

        # Additional state info (8)
        obs[37] = self.step_count / self.max_steps  # Progress
        obs[38] = self.steps_taken  # Steps taken
        obs[39] = self.total_distance  # Distance traveled
        obs[40] = float(self.fell)  # Fell flag

        # Placeholder for phase info (5)
        obs[41:45] = 0.0

        return obs

    def _check_termination(self) -> Tuple[bool, str]:
        """Check if episode should terminate."""
        if self.base_pose is None:
            return False, ""

        # Fell detection
        z = self.base_pose.pose.position.z
        if z < 0.3:  # Robot too low
            return True, "fell_low"

        # Tipping detection (roll/pitch > 45°)
        if self.imu_msg is not None:
            quat = self.imu_msg.orientation
            # Simple roll/pitch check (approximate)
            roll = np.arctan2(
                2.0 * (quat.w * quat.x + quat.y * quat.z),
                1.0 - 2.0 * (quat.x**2 + quat.y**2)
            )
            pitch = np.arcsin(2.0 * (quat.w * quat.y - quat.z * quat.x))

            if abs(roll) > 0.785 or abs(pitch) > 0.785:  # 45°
                return True, "tipped"

        # Max steps reached
        if self.step_count >= self.max_steps:
            return True, "timeout"

        return False, ""

    def _compute_reward(self) -> float:
        """Compute reward for current state."""
        reward = 0.0

        if self.base_pose is None:
            return -10.0

        # 1. Forward progress reward
        current_x = self.base_pose.pose.position.x
        dx = current_x - self.last_com_x
        if dx > 0:
            reward += dx * 100.0  # Scale to make significant
            self.total_distance += dx
        self.last_com_x = current_x

        # 2. Upright reward (penalize tipping)
        z = self.base_pose.pose.position.z
        if z > 0.45:  # Good height
            reward += 0.5

        # 3. Step count reward
        # (Would need phase tracking from WBC controller)
        # reward += self.steps_taken * 2.0

        # 4. Stability reward (penalize high velocities)
        if self.joint_state is not None and len(self.joint_state.velocity) >= 12:
            vel_penalty = np.sum(np.abs(self.joint_state.velocity[:12]))
            reward -= vel_penalty * 0.01

        # 5. Fell penalty
        if self.fell:
            reward -= 100.0

        # 6. Bonus for surviving
        reward += 0.1

        return reward

    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict]:
        """Reset environment for new episode."""
        super().reset(seed=seed)

        # Reset state
        self.step_count = 0
        self.fell = False
        self.steps_taken = 0
        self.total_distance = 0.0
        self.last_com_x = 0.0
        self.joint_state = None
        self.imu_msg = None
        self.base_pose = None

        # Apply new random or given config
        if options and 'params' in options:
            params = options['params']
        else:
            # Sample random params from action space
            params = self.action_space.sample()

        self._apply_config(params)

        # Restart simulation
        self._start_simulation()

        # Wait for first observations
        timeout = 5.0
        start = time.time()
        while (self.joint_state is None or self.base_pose is None):
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if time.time() - start > timeout:
                break

        obs = self._get_observation()
        info = {}

        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute one environment step."""
        # Note: action is applied at reset, not during step
        # (Config doesn't change mid-episode)

        # Spin ROS2 to get new messages
        rclpy.spin_once(self.node, timeout_sec=0.01)

        self.step_count += 1

        # Get observation
        obs = self._get_observation()

        # Check termination
        terminated, reason = self._check_termination()
        if terminated:
            self.fell = (reason in ['fell_low', 'tipped'])

        # Compute reward
        reward = self._compute_reward()

        truncated = False
        info = {
            'step_count': self.step_count,
            'steps_taken': self.steps_taken,
            'distance': self.total_distance,
            'fell': self.fell,
            'reason': reason if terminated else '',
        }

        return obs, reward, terminated, truncated, info

    def close(self):
        """Cleanup environment."""
        self._stop_simulation()

        # Restore original config
        if self.backup_config is not None:
            with open(self.config_path, 'w') as f:
                yaml.dump(self.backup_config, f, default_flow_style=False)

        super().close()


if __name__ == "__main__":
    # Test environment
    env = WbcTuningEnv()

    print("Testing WBC Tuning Environment...")
    print(f"Action space: {env.action_space}")
    print(f"Observation space: {env.observation_space}")

    obs, info = env.reset()
    print(f"Initial observation shape: {obs.shape}")

    for i in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"Step {i}: reward={reward:.2f}, terminated={terminated}")

        if terminated or truncated:
            break

    env.close()
    print("Test complete!")
