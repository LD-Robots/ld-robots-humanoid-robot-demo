#!/usr/bin/env python3
"""
Gymnasium environment for PPO-based WBC parameter tuning.
Wraps MuJoCo simulation and WBC controller for RL training.
"""

import gymnasium as gym
import numpy as np
from typing import Dict, Tuple, Optional
import subprocess
import signal
import os
import time
import yaml
import copy
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PoseStamped


class WbcTuningEnv(gym.Env):
    """RL environment for tuning WBC walking parameters."""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(
        self,
        config_path: str = None,
        max_steps: int = 1000,  # 10 seconds @ 100Hz
        warmup_steps: int = 50,
        allow_timeout: bool = False,
        render_mode: Optional[str] = None,
        namespace: str = "",  # ROS2 namespace for parallel environments
    ):
        super().__init__()

        self.max_steps = max_steps
        self.warmup_steps = warmup_steps
        self.allow_timeout = allow_timeout
        self.render_mode = render_mode
        self.step_count = 0
        self.namespace = namespace  # Store namespace

        # ROS2 setup
        if not rclpy.ok():
            rclpy.init()

        # Paths
        self.pkg_path = Path(__file__).parent.parent

        # Use unique config file for each namespace to avoid conflicts
        if namespace:
            config_dir = self.pkg_path / "config" / "parallel"
            config_dir.mkdir(exist_ok=True)
            self.config_path = str(config_dir / f"wbc_controller_{namespace}.yaml")
            # Copy base config if it doesn't exist
            base_config = config_path or str(self.pkg_path / "config" / "wbc_controller.yaml")
            if not Path(self.config_path).exists():
                import shutil
                shutil.copy(base_config, self.config_path)
            # Ensure namespaced node key and relative topics for namespace support
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f) or {}
            desired_node_key = f"/{namespace}/wbc_controller"
            node_key_updated = False
            if desired_node_key not in config:
                if 'wbc_controller' in config:
                    config[desired_node_key] = config.pop('wbc_controller')
                    node_key_updated = True
                elif config:
                    first_key = next(iter(config.keys()))
                    config[desired_node_key] = config.pop(first_key)
                    node_key_updated = True
                else:
                    config[desired_node_key] = {'ros__parameters': {}}
                    node_key_updated = True
            wbc_params = config[desired_node_key].setdefault('ros__parameters', {})
            # Remove leading '/' to make topics relative (will be namespaced)
            desired_topics = {
                'target_positions_topic': 'target_positions',
                'joint_states_topic': 'joint_states',
                'imu_topic': 'imu/data',
                'base_pose_topic': 'pelvis/pose',
            }
            updated = False
            for key, value in desired_topics.items():
                if wbc_params.get(key) != value:
                    wbc_params[key] = value
                    updated = True
            if updated or node_key_updated:
                with open(self.config_path, 'w') as f:
                    yaml.dump(config, f, default_flow_style=False, sort_keys=False)
        else:
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
        self.last_roll = None
        self.last_pitch = None
        self.no_data = False
        self._logged_joint = False
        self._logged_imu = False
        self._logged_pose = False
        self._first_episode = True
        self.last_config_snapshot = None

        # Track milestone achievements (one-time bonuses)
        self.milestones_achieved = set()

        # Retry logic for early falls during warmup
        self.max_warmup_retries = 3  # Maximum retries if robot falls during warmup
        self.current_params = None   # Store current params for retry

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
        _config, wbc_params, _node_key = self._load_config_params()

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

    def _load_config_params(self):
        """Load config and return (config, ros__parameters dict, node key)."""
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f) or {}

        if 'wbc_controller' in config:
            node_key = 'wbc_controller'
            params = config[node_key].get('ros__parameters', {})
            return config, params, node_key

        for node_key, node_val in config.items():
            if not isinstance(node_val, dict):
                continue
            params = node_val.get('ros__parameters')
            if isinstance(params, dict):
                return config, params, node_key

        raise KeyError("Missing wbc_controller ros__parameters in config")

    def _apply_config(self, params: np.ndarray):
        """
        Apply RL action to WBC config file.
        Params are MULTIPLIERS (0.7-1.3) applied to baseline values.
        """
        # Compute actual values: baseline * multiplier
        actual_values = self.baseline_params * params

        # Read current config
        config, wbc_params, node_key = self._load_config_params()

        # Backup on first call
        if self.backup_config is None:
            self.backup_config = config.copy()

        # Update ALL tunable parameters (39 params)

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

        config[node_key]['ros__parameters'] = wbc_params
        # Write updated config
        with open(self.config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)
        self.last_config_snapshot = copy.deepcopy(config)

    def _start_simulation(self):
        """Launch ROS2 simulation with current config and unique namespace."""
        # Kill any existing simulation
        self._stop_simulation()

        # Determine if viewer should be enabled based on render_mode
        use_viewer = 'true' if self.render_mode == 'human' else 'false'

        # Build launch command with namespace support
        publish_clock = 'true' if (not self.namespace or self.namespace == 'robot_0') else 'false'
        launch_cmd = [
            'ros2', 'launch',
            'wbc_pinocchio_controller', 'wbc_full_mujoco.launch.py',
            f'use_viewer:={use_viewer}',
            'use_rviz:=false',
            f'wbc_config:={self.config_path}',
            f'publish_clock:={publish_clock}',
        ]

        # Add namespace parameter if specified
        if self.namespace:
            launch_cmd.append(f'namespace:={self.namespace}')

        # Launch the simulation (capture output to log file for debugging)
        log_dir = Path("/tmp/wbc_training_logs")
        log_dir.mkdir(exist_ok=True)
        log_file = log_dir / f"{self.namespace or 'default'}_launch.log"
        log_f = open(log_file, 'w')

        self.sim_process = subprocess.Popen(
            launch_cmd,
            stdout=log_f,
            stderr=subprocess.STDOUT,  # Redirect stderr to stdout
            preexec_fn=os.setsid,
        )

        # Wait for nodes to start (longer timeout for parallel environments)
        # Increased timeout to ensure all ROS2 nodes are ready
        wait_time = 8.0 if self.namespace else 5.0
        time.sleep(wait_time)
        if self.sim_process.poll() is not None:
            try:
                stdout, stderr = self.sim_process.communicate(timeout=1)
            except Exception:
                stdout, stderr = b"", b""
            if stdout:
                print("WARN: ROS2 launch stdout (first 2000 chars):")
                print(stdout.decode(errors="replace")[:2000])
            if stderr:
                print("WARN: ROS2 launch stderr (first 2000 chars):")
                print(stderr.decode(errors="replace")[:2000])
            print("WARN: ROS2 launch exited early; no simulation process running")
            self.no_data = True

        # Create ROS2 subscribers with namespaced topics
        node_name = f'wbc_rl_env_{self.namespace}' if self.namespace else 'wbc_rl_env'
        self.node = rclpy.create_node(node_name)

        # Topic names - use namespace if specified for proper isolation
        if self.namespace:
            joint_topic = f'/{self.namespace}/joint_states'
            imu_topic = f'/{self.namespace}/imu/data'
            pose_topic = f'/{self.namespace}/pelvis/pose'
        else:
            joint_topic = '/joint_states'
            imu_topic = '/imu/data'
            pose_topic = '/pelvis/pose'

        self.joint_sub = self.node.create_subscription(
            JointState, joint_topic, self._joint_callback, qos_profile_sensor_data
        )
        self.imu_sub = self.node.create_subscription(
            Imu, imu_topic, self._imu_callback, qos_profile_sensor_data
        )
        self.pose_sub = self.node.create_subscription(
            PoseStamped, pose_topic, self._pose_callback, qos_profile_sensor_data
        )

    def _stop_simulation(self):
        """Stop ROS2 simulation."""
        if self.sim_process is not None:
            try:
                os.killpg(os.getpgid(self.sim_process.pid), signal.SIGTERM)
                self.sim_process.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.sim_process.pid), signal.SIGKILL)
                except Exception:
                    pass
            self.sim_process = None

        if hasattr(self, 'node'):
            self.node.destroy_node()

    def _joint_callback(self, msg: JointState):
        self.joint_state = msg
        self.no_data = False
        if not self._logged_joint:
            print("INFO: Received first /joint_states message")
            self._logged_joint = True

    def _imu_callback(self, msg: Imu):
        self.imu_msg = msg
        self.no_data = False
        if not self._logged_imu:
            print("INFO: Received first /imu/data message")
            self._logged_imu = True

    def _pose_callback(self, msg: PoseStamped):
        self.base_pose = msg
        self.no_data = False
        if not self._logged_pose:
            print("INFO: Received first /pelvis/pose message")
            self._logged_pose = True

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
        if self.no_data and self.step_count > self.warmup_steps:
            return True, "no_data"

        if self.step_count <= self.warmup_steps:
            return False, ""

        if self.base_pose is None or self.joint_state is None:
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

        # Max steps reached (optional)
        if self.allow_timeout and self.step_count >= self.max_steps:
            return True, "timeout"

        return False, ""

    def _compute_reward(self) -> float:
        """
        Compute reward for current state.

        Reward structure optimized to encourage:
        1. Forward progress (main objective)
        2. Stability and upright posture
        3. Straight walking without drift
        4. Survival without excessive penalties
        """
        if self.step_count <= self.warmup_steps:
            return 0.0

        reward = 0.0

        if self.base_pose is None:
            return -5.0  # Reduced from -10.0

        # 1. Forward progress reward (PRIMARY OBJECTIVE)
        current_x = self.base_pose.pose.position.x
        dx = current_x - self.last_com_x
        if dx > 0:
            # Increased weight: forward progress is the main goal
            reward += dx * 200.0  # Increased from 100.0
            reward += 0.2  # Small bonus to keep forward motion preferred
            self.total_distance += dx
        elif dx < -0.001:  # Small penalty for moving backwards
            reward -= abs(dx) * 50.0
        self.last_com_x = current_x

        # 2. Upright posture reward (INCREASED)
        z = self.base_pose.pose.position.z
        if z > 0.45:  # Good height
            reward += 2.0  # Increased from 0.5
        elif z > 0.40:  # Still acceptable
            reward += 1.0
        else:  # Getting too low
            reward -= (0.40 - z) * 10.0

        # 3. Orientation stability (roll/pitch)
        if self.imu_msg is not None:
            quat = self.imu_msg.orientation
            # Calculate roll and pitch
            roll = np.arctan2(
                2.0 * (quat.w * quat.x + quat.y * quat.z),
                1.0 - 2.0 * (quat.x**2 + quat.y**2)
            )
            pitch = np.arcsin(np.clip(2.0 * (quat.w * quat.y - quat.z * quat.x), -1.0, 1.0))

            # Reward for staying upright (small deviations OK)
            roll_penalty = abs(roll) * 2.0 if abs(roll) > 0.1 else 0.0
            pitch_penalty = abs(pitch) * 2.0 if abs(pitch) > 0.1 else 0.0
            reward -= (roll_penalty + pitch_penalty)

            # Penalize oscillations using angular velocity and roll/pitch changes
            roll_rate = float(self.imu_msg.angular_velocity.x)
            pitch_rate = float(self.imu_msg.angular_velocity.y)
            reward -= (abs(roll_rate) + abs(pitch_rate)) * 0.2
            if self.last_roll is not None and self.last_pitch is not None:
                reward -= (abs(roll - self.last_roll) + abs(pitch - self.last_pitch)) * 0.2
            self.last_roll = roll
            self.last_pitch = pitch

            # Bonus for being very stable
            if abs(roll) < 0.05 and abs(pitch) < 0.05:
                reward += 0.5

        # 4. Joint velocity smoothness (MODIFIED - don't penalize walking)
        if self.joint_state is not None and len(self.joint_state.velocity) >= 12:
            # Only penalize EXCESSIVE velocities (> 2 rad/s)
            vel_array = np.array(self.joint_state.velocity[:12])
            excessive_vel = np.sum(np.maximum(0, np.abs(vel_array) - 2.0))
            reward -= excessive_vel * 0.5  # Reduced penalty

        # 5. Straight-line bonus (lateral drift and yaw)
        y = self.base_pose.pose.position.y
        reward -= abs(y) * 3.0  # Reduced from 5.0

        quat = self.base_pose.pose.orientation
        yaw = np.arctan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y**2 + quat.z**2)
        )
        reward -= abs(yaw) * 1.0

        # Bonus for staying centered and aligned
        if abs(y) < 0.05 and abs(yaw) < 0.1:
            reward += 1.0

        # 6. Fell penalty (REDUCED - don't dominate the reward)
        if self.fell:
            reward -= 20.0  # Reduced from 100.0 - still bad but not overwhelming

        # 7. Survival bonus (INCREASED - encourage longevity)
        reward += 0.5  # Increased from 0.1

        # 8. Distance milestone bonuses (ONE-TIME only!)
        # Check and award each milestone only once per episode
        if self.total_distance > 0.5 and 0.5 not in self.milestones_achieved:
            reward += 10.0
            self.milestones_achieved.add(0.5)
            if self.verbose if hasattr(self, 'verbose') else False:
                print(f"🎯 Milestone: 0.5m reached!")

        if self.total_distance > 1.0 and 1.0 not in self.milestones_achieved:
            reward += 25.0
            self.milestones_achieved.add(1.0)
            if self.verbose if hasattr(self, 'verbose') else False:
                print(f"🎯 Milestone: 1.0m reached!")

        if self.total_distance > 2.0 and 2.0 not in self.milestones_achieved:
            reward += 50.0
            self.milestones_achieved.add(2.0)
            if self.verbose if hasattr(self, 'verbose') else False:
                print(f"🎯 Milestone: 2.0m reached!")

        return reward

    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict]:
        """Reset environment for new episode with retry logic for early falls."""
        super().reset(seed=seed)

        # Determine params for this episode
        if options and 'params' in options:
            params = options['params']
        elif self._first_episode:
            params = np.ones(self.action_space.shape, dtype=np.float32)
            self._first_episode = False
        else:
            # Sample random params from action space
            params = self.action_space.sample()

        # Store params for potential retry
        self.current_params = params.copy()

        # Try to reset with retry logic for warmup failures
        for retry in range(self.max_warmup_retries):
            # Reset state
            self.step_count = 0
            self.fell = False
            self.steps_taken = 0
            self.total_distance = 0.0
            self.last_com_x = 0.0
            self.last_roll = None
            self.last_pitch = None
            self.joint_state = None
            self.imu_msg = None
            self.base_pose = None
            self.no_data = False
            self._logged_joint = False
            self._logged_imu = False
            self._logged_pose = False
            self.milestones_achieved = set()  # Reset milestones for new episode

            # Apply config
            self._apply_config(self.current_params)

            # Restart simulation
            self._start_simulation()

            # Wait for first observations with longer timeout for parallel environments
            timeout = 30.0 if self.namespace else 20.0
            start = time.time()
            print(f"[{self.namespace or 'default'}] Waiting for ROS2 data (timeout: {timeout}s)...")

            while (self.joint_state is None or self.base_pose is None):
                rclpy.spin_once(self.node, timeout_sec=0.01)
                if time.time() - start > timeout:
                    break

                # Show progress every 5 seconds
                elapsed = time.time() - start
                if int(elapsed) % 5 == 0 and elapsed > 0:
                    print(f"[{self.namespace or 'default'}] Still waiting... ({elapsed:.0f}s / {timeout}s)")

            if self.joint_state is None or self.base_pose is None:
                print(
                    f"WARN [{self.namespace or 'default'}]: No ROS2 data after {timeout}s (retry {retry+1}/{self.max_warmup_retries})"
                    f"\n  joint_state: {self.joint_state is not None}"
                    f"\n  base_pose: {self.base_pose is not None}"
                    f"\n  imu: {self.imu_msg is not None}"
                )
                if self.sim_process is not None and self.sim_process.poll() is not None:
                    self.no_data = True
                    print(f"WARN [{self.namespace or 'default'}]: Simulation process died!")

                # Stop and retry
                self._stop_simulation()
                time.sleep(2.0)
                continue  # Retry

            print(f"✓ [{self.namespace or 'default'}] ROS2 data received after {time.time() - start:.1f}s")

            # Check if robot falls during warmup period
            print(f"[{self.namespace or 'default'}] Checking warmup stability ({self.warmup_steps} steps)...")
            warmup_ok = self._check_warmup_stability()

            if warmup_ok:
                # Success! Robot is stable during warmup
                print(f"✅ [{self.namespace or 'default'}] Warmup successful!")
                break
            else:
                if retry < self.max_warmup_retries - 1:
                    print(f"⚠️  [{self.namespace or 'default'}] Robot fell during warmup!")
                    print(f"   Retrying ({retry+2}/{self.max_warmup_retries})...")
                    self._stop_simulation()
                    time.sleep(2.0)  # Pause before retry
                else:
                    print(f"❌ [{self.namespace or 'default'}] Robot failed warmup after {self.max_warmup_retries} retries")
                    print(f"   Continuing anyway - will likely fall quickly...")

        obs = self._get_observation()

        # Track actual number of retries attempted
        actual_retries = retry + 1 if warmup_ok or retry < self.max_warmup_retries - 1 else self.max_warmup_retries
        info = {
            'warmup_retries': actual_retries,
            'warmup_success': warmup_ok if 'warmup_ok' in locals() else False
        }

        return obs, info

    def _check_warmup_stability(self) -> bool:
        """
        Check if robot remains stable during warmup period.
        Returns True if stable, False if fell.
        """
        warmup_start = time.time()
        warmup_duration = self.warmup_steps / 100.0  # Assuming 100Hz control

        while time.time() - warmup_start < warmup_duration:
            rclpy.spin_once(self.node, timeout_sec=0.01)

            # Check if robot fell
            if self.base_pose is not None:
                z = self.base_pose.pose.position.z
                if z < 0.2:  # Fell down
                    return False

                # Check orientation
                if self.imu_msg is not None:
                    quat = self.imu_msg.orientation
                    roll = np.arctan2(
                        2.0 * (quat.w * quat.x + quat.y * quat.z),
                        1.0 - 2.0 * (quat.x**2 + quat.y**2)
                    )
                    pitch = np.arcsin(np.clip(2.0 * (quat.w * quat.y - quat.z * quat.x), -1.0, 1.0))

                    if abs(roll) > 0.5 or abs(pitch) > 0.5:  # Tipped over
                        return False

            time.sleep(0.01)  # 100Hz check rate

        # Warmup completed successfully
        self.step_count = self.warmup_steps  # Mark warmup as complete
        return True

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
            # Stop simulation early to avoid piling up processes
            self._stop_simulation()

        # Compute reward
        reward = self._compute_reward()

        truncated = False
        info = {
            'step_count': self.step_count,
            'steps_taken': self.steps_taken,
            'distance': self.total_distance,
            'fell': self.fell,
            'reason': reason if terminated else '',
            'obs_valid': (self.joint_state is not None and self.base_pose is not None),
            'config_path': self.config_path,
        }
        if terminated:
            info['config_snapshot'] = copy.deepcopy(self.last_config_snapshot)

        return obs, reward, terminated, truncated, info

    def close(self):
        """Cleanup environment."""
        self._stop_simulation()

        # Restore original config
        if self.backup_config is not None:
            with open(self.config_path, 'w') as f:
                yaml.dump(self.backup_config, f, default_flow_style=False)

        super().close()


class WbcGainsTuningEnv(WbcTuningEnv):
    """RL environment that tunes only balance/IMU kp/kd gains."""

    GAIN_KEYS = [
        "balance_kp_pitch",
        "balance_kd_pitch",
        "imu_kp_pitch",
        "imu_kd_pitch",
        "balance_kp_roll",
        "balance_kd_roll",
        "imu_kp_roll",
        "imu_kd_roll",
    ]
    GAIN_INDICES = [23, 24, 25, 26, 27, 28, 29, 30]

    def __init__(
        self,
        config_path: str = None,
        max_steps: int = 1000,
        warmup_steps: int = 50,
        allow_timeout: bool = False,
        render_mode: Optional[str] = None,
        namespace: str = "",  # ROS2 namespace for parallel environments
    ):
        super().__init__(
            config_path=config_path,
            max_steps=max_steps,
            warmup_steps=warmup_steps,
            allow_timeout=allow_timeout,
            render_mode=render_mode,
            namespace=namespace,  # Pass namespace to parent
        )
        # Increased action space for better exploration
        # Was: 0.9-1.1 (±10%) → Now: 0.7-1.3 (±30%)
        # This allows the agent to explore more parameter variations
        self.action_space = gym.spaces.Box(
            low=np.full(len(self.GAIN_KEYS), 0.7, dtype=np.float32),
            high=np.full(len(self.GAIN_KEYS), 1.3, dtype=np.float32),
            dtype=np.float32,
        )

    def _apply_config(self, params: np.ndarray):
        """Apply only gain parameters as multipliers to the baseline."""
        actual_values = self.baseline_params[self.GAIN_INDICES] * params

        config, wbc_params, node_key = self._load_config_params()

        if self.backup_config is None:
            self.backup_config = config.copy()

        for key, value in zip(self.GAIN_KEYS, actual_values):
            wbc_params[key] = float(value)

        config[node_key]['ros__parameters'] = wbc_params
        with open(self.config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)
        self.last_config_snapshot = copy.deepcopy(config)


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
