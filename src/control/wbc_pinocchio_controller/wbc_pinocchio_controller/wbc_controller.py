#!/usr/bin/env python3
"""
Quasi-static WBC controller for MuJoCo humanoid.
Uses Pinocchio for CoM estimation and optional Crocoddyl hooks.
Commands joint positions over /joint_commands (Float64MultiArray).
"""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from .config_loader import load_wbc_config
from .joint_mapping_loader import load_initial_pose

try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except Exception:
    PINOCCHIO_AVAILABLE = False


class Phase(Enum):
    STABILIZE = 0
    SHIFT_TO_LEFT = 1
    RIGHT_SWING = 2
    RIGHT_LAND = 3
    SHIFT_TO_RIGHT = 4
    LEFT_SWING = 5
    LEFT_LAND = 6


@dataclass
class RobotState:
    joint_positions: Dict[str, float]
    joint_velocities: Dict[str, float]
    com: Optional[np.ndarray]
    com_vel: Optional[np.ndarray]


class WbcPinocchioController(Node):
    def __init__(self):
        super().__init__('wbc_controller')

        self.cfg = load_wbc_config(self)

        self.base_pose = load_initial_pose(
            self.cfg.initial_pose_yaml,
            self.cfg.initial_pose_key,
            logger=self.get_logger(),
        )
        self.command_joint_names = list(self.base_pose.keys())
        if self.command_joint_names:
            self.get_logger().info(f'Loaded base pose joints: {len(self.command_joint_names)}')
            self.get_logger().info(f'Initial pose YAML: {self.cfg.initial_pose_yaml}')
            self.get_logger().info(f'Initial pose key: {self.cfg.initial_pose_key}')
            if 'left_hip_pitch_joint' in self.base_pose:
                self.get_logger().info(
                    f"Initial left_hip_pitch_joint={self.base_pose['left_hip_pitch_joint']:.3f}"
                )
        else:
            self.get_logger().warn(
                f"Initial pose empty. YAML={self.cfg.initial_pose_yaml} key={self.cfg.initial_pose_key}"
            )

        self.pin_model = None
        self.pin_data = None
        self.pin_joint_map = {}
        self.has_floating_base = False
        if PINOCCHIO_AVAILABLE and self.cfg.urdf_path:
            try:
                if self.cfg.urdf_path.endswith('.xml'):
                    self.pin_model = pin.buildModelFromMJCF(self.cfg.urdf_path)
                else:
                    self.pin_model = pin.buildModelFromUrdf(self.cfg.urdf_path)
                self.pin_data = self.pin_model.createData()
                if len(self.pin_model.joints) > 1:
                    self.has_floating_base = self.pin_model.joints[1].nq == 7
                self.get_logger().info(f'Floating base: {self.has_floating_base}')
                for name in self.pin_model.names:
                    if name == 'universe':
                        continue
                    joint_id = self.pin_model.getJointId(name)
                    if joint_id >= 0:
                        idx = self.pin_model.joints[joint_id].idx_q
                        self.pin_joint_map[name] = idx
                self.get_logger().info(f'Pinocchio model loaded: {self.pin_model.nq} q')
                self.get_logger().info(
                    f'CoM frame: {"world" if self.cfg.com_in_world else "model"}'
                )
            except Exception as exc:
                self.get_logger().warn(f'Failed to load Pinocchio model: {exc}')

        self.joint_state = None
        self.imu_msg = None
        self.base_pose_msg = None
        self.filtered_commands = {}
        self.ankle_pitch_filtered = 0.0
        self.ankle_roll_filtered = 0.0
        self.hip_pitch_filtered = 0.0
        self.hip_roll_filtered = 0.0
        self.hold_active = False
        self.hold_targets = {}
        self.stable_since = None
        self.last_com_error_x = 0.0
        self.last_com_error_y = 0.0
        self.last_com_vel_x = 0.0
        self.last_com_vel_y = 0.0
        self.phase_time_offset = 0.0
        self.last_control_time = None
        self.phase_hold_accum = 0.0

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.cfg.joint_states_topic,
            self._joint_state_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            self.cfg.imu_topic,
            self._imu_callback,
            10
        )
        self.base_pose_sub = self.create_subscription(
            PoseStamped,
            self.cfg.base_pose_topic,
            self._base_pose_callback,
            10
        )
        self.target_pub = self.create_publisher(JointState, self.cfg.target_positions_topic, 10)
        self.com_pub = self.create_publisher(PointStamped, 'com_position', 10)
        self.zmp_pub = self.create_publisher(PointStamped, 'zmp_position', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'balance_markers', 10)

        self.start_time = None
        self.phase = Phase.STABILIZE
        self.last_log_time = None

        self.timer_started = False
        if self.cfg.use_crocoddyl:
            self.get_logger().warn('Crocoddyl integration not implemented yet; running quasi-static WBC only.')

    def _load_actuator_order(self, model_xml_path: str) -> List[str]:
        if not model_xml_path:
            return []
        try:
            tree = ET.parse(model_xml_path)
            root = tree.getroot()
        except Exception as exc:
            self.get_logger().warn(f'Failed to parse model XML: {exc}')
            return []

        actuators = []
        actuator_root = root.find('actuator')
        if actuator_root is None:
            return []

        for child in actuator_root:
            joint_name = child.attrib.get('joint')
            if joint_name:
                actuators.append(joint_name)
        return actuators

    def _imu_callback(self, msg: Imu):
        self.imu_msg = msg

    def _base_pose_callback(self, msg: PoseStamped):
        self.base_pose_msg = msg

    def _build_state(self) -> Optional[RobotState]:
        if self.joint_state is None:
            return None

        positions = {}
        velocities = {}
        for idx, name in enumerate(self.joint_state.name):
            positions[name] = self.joint_state.position[idx]
            if idx < len(self.joint_state.velocity):
                velocities[name] = self.joint_state.velocity[idx]

        com = None
        com_vel = None
        if self.pin_model is not None and self.pin_data is not None:
            q = np.zeros(self.pin_model.nq)
            dq = np.zeros(self.pin_model.nv)
            for joint_name, q_idx in self.pin_joint_map.items():

                if joint_name == 'floating_base_joint' and self.cfg.com_in_world:
                    continue

                if joint_name in positions:
                    q[q_idx] = positions[joint_name]
                if joint_name in velocities:
                    dq[q_idx] = velocities[joint_name]
            if self.has_floating_base and self.cfg.com_in_world and self.base_pose_msg is not None:
                pose = self.base_pose_msg.pose
                q[0:3] = [pose.position.x, pose.position.y, pose.position.z]

                q[3:7] = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

                quat = np.array(
                    [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                    dtype=float
                )
                norm = np.linalg.norm(quat)
                if norm > 1e-9:
                    quat = quat / norm
                q[3:7] = quat
            try:
                pin.forwardKinematics(self.pin_model, self.pin_data, q, dq)
                pin.centerOfMass(self.pin_model, self.pin_data, q, dq)
                com = self.pin_data.com[0].copy()
                com_vel = self.pin_data.vcom[0].copy()
            except Exception as exc:
                self.get_logger().warn(f'Pinocchio computation failed: {exc}', throttle_duration_sec=5.0)

        return RobotState(positions, velocities, com, com_vel)

    def _quat_to_rpy(self, quat: Imu) -> Tuple[float, float, float]:
        q = quat.orientation
        w, x, y, z = q.w, q.x, q.y, q.z
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * (np.pi / 2.0)
        else:
            pitch = np.arcsin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def _quat_to_rot(self, quat) -> np.ndarray:
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ])
    def _phase_progress(self, elapsed: float) -> Tuple[Phase, float]:
        if elapsed < self.cfg.stabilize_duration:
            return Phase.STABILIZE, elapsed / max(self.cfg.stabilize_duration, 1e-6)

        cycle_time = elapsed - self.cfg.stabilize_duration
        phase_time = self.cfg.step_duration
        if phase_time <= 0.0:
            return Phase.STABILIZE, 0.0

        cycle_pos = cycle_time % (phase_time * 2.0)
        half = phase_time

        if cycle_pos < half:
            phase_elapsed = cycle_pos
            phase = self._phase_from_cycle(phase_elapsed, left_support=True)
            progress = phase_elapsed / half
        else:
            phase_elapsed = cycle_pos - half
            phase = self._phase_from_cycle(phase_elapsed, left_support=False)
            progress = phase_elapsed / half

        return phase, progress

    def _phase_from_cycle(self, t: float, left_support: bool) -> Phase:
        shift = self.cfg.step_duration * 0.2
        swing = self.cfg.step_duration * 0.5
        land = self.cfg.step_duration * 0.3
        if t < shift:
            return Phase.SHIFT_TO_LEFT if left_support else Phase.SHIFT_TO_RIGHT
        if t < shift + swing:
            return Phase.RIGHT_SWING if left_support else Phase.LEFT_SWING
        return Phase.RIGHT_LAND if left_support else Phase.LEFT_LAND

    def _build_targets(self, state: RobotState, phase: Phase, progress: float) -> Dict[str, float]:
        targets: Dict[str, float] = dict(self.base_pose)

        if not targets:
            targets['left_hip_roll_joint'] = self.cfg.stance_width / 2.0
            targets['right_hip_roll_joint'] = -self.cfg.stance_width / 2.0
            targets['left_knee_joint'] = self.cfg.support_knee_bend
            targets['right_knee_joint'] = self.cfg.support_knee_bend
            targets['left_hip_pitch_joint'] = -self.cfg.support_hip_pitch
            targets['right_hip_pitch_joint'] = -self.cfg.support_hip_pitch
            targets['left_ankle_pitch_joint'] = self.cfg.support_ankle_pitch
            targets['right_ankle_pitch_joint'] = self.cfg.support_ankle_pitch

        # Swing definitions
        swing_progress = np.sin(np.pi * progress)
        if phase in (Phase.RIGHT_SWING, Phase.RIGHT_LAND):
            targets['right_knee_joint'] = self.cfg.support_knee_bend + self.cfg.swing_knee_bend * swing_progress
            targets['right_hip_pitch_joint'] = -self.cfg.swing_hip_pitch * swing_progress
            targets['right_ankle_pitch_joint'] = -self.cfg.swing_ankle_pitch * swing_progress
        elif phase in (Phase.LEFT_SWING, Phase.LEFT_LAND):
            targets['left_knee_joint'] = self.cfg.support_knee_bend + self.cfg.swing_knee_bend * swing_progress
            targets['left_hip_pitch_joint'] = -self.cfg.swing_hip_pitch * swing_progress
            targets['left_ankle_pitch_joint'] = -self.cfg.swing_ankle_pitch * swing_progress

        # Lateral weight shift toward support leg
        shift_progress = 0.0
        support_sign = 0.0
        if phase == Phase.SHIFT_TO_LEFT:
            shift_progress = progress
            support_sign = 1.0
        elif phase == Phase.SHIFT_TO_RIGHT:
            shift_progress = progress
            support_sign = -1.0
        elif phase in (Phase.RIGHT_SWING, Phase.RIGHT_LAND):
            shift_progress = 1.0 if phase == Phase.RIGHT_SWING else (1.0 - progress)
            support_sign = 1.0
        elif phase in (Phase.LEFT_SWING, Phase.LEFT_LAND):
            shift_progress = 1.0 if phase == Phase.LEFT_SWING else (1.0 - progress)
            support_sign = -1.0

        if support_sign != 0.0:
            hip_roll_offset = support_sign * self.cfg.hip_roll_shift * shift_progress
            targets['left_hip_roll_joint'] = targets.get('left_hip_roll_joint', 0.0) + hip_roll_offset
            targets['right_hip_roll_joint'] = targets.get('right_hip_roll_joint', 0.0) - hip_roll_offset

        # Balance corrections (CoM-based, similar to pinocchio_balance_control)
        com_error_x = 0.0
        com_error_y = 0.0
        com_vel_x = 0.0
        com_vel_y = 0.0
        support_center_x = 0.0
        support_center_y = 0.0
        if self.cfg.com_in_world and self.base_pose_msg is not None:
            support_center_x = float(self.base_pose_msg.pose.position.x)
            support_center_y = float(self.base_pose_msg.pose.position.y)
        if state.com is not None and state.com_vel is not None:
            com_error_x = state.com[0] - support_center_x
            com_error_y = state.com[1] - support_center_y
            com_vel_x = state.com_vel[0]
            com_vel_y = state.com_vel[1]

        self.last_com_error_x = com_error_x
        self.last_com_error_y = com_error_y
        self.last_com_vel_x = com_vel_x
        self.last_com_vel_y = com_vel_y

        if abs(com_error_x) < self.cfg.com_deadzone:
            com_error_x = 0.0
        if abs(com_error_y) < self.cfg.com_deadzone:
            com_error_y = 0.0

        support_limit_x = max(self.cfg.foot_length * 0.5, 1e-3)
        support_limit_y = max(self.cfg.foot_width * 0.5, 1e-3)
        com_margin_x = abs(com_error_x) / support_limit_x
        com_margin_y = abs(com_error_y) / support_limit_y

        com_predicted_x = com_error_x + com_vel_x * self.cfg.prediction_time
        com_predicted_y = com_error_y + com_vel_y * self.cfg.prediction_time
        com_error_blend_x = 0.7 * com_error_x + 0.3 * com_predicted_x
        com_error_blend_y = 0.7 * com_error_y + 0.3 * com_predicted_y

        correction_x = -(self.cfg.balance_kp_pitch * com_error_blend_x + self.cfg.balance_kd_pitch * com_vel_x)
        correction_y = -(self.cfg.balance_kp_roll * com_error_blend_y + self.cfg.balance_kd_roll * com_vel_y)

        com_tilt_mag = float(np.sqrt(com_margin_x**2 + com_margin_y**2))
        hip_ratio = float(np.tanh(com_tilt_mag / 0.3))
        ankle_ratio = 1.0 - 0.5 * hip_ratio

        ankle_pitch_raw = np.clip(-ankle_ratio * correction_x, -self.cfg.ankle_pitch_limit, self.cfg.ankle_pitch_limit)
        ankle_roll_raw = np.clip(-ankle_ratio * correction_y, -self.cfg.ankle_roll_limit, self.cfg.ankle_roll_limit)
        hip_pitch_raw = np.clip(-hip_ratio * correction_x, -self.cfg.hip_pitch_limit, self.cfg.hip_pitch_limit)
        hip_roll_raw = np.clip(-hip_ratio * correction_y, -self.cfg.hip_roll_limit, self.cfg.hip_roll_limit)

        self.ankle_pitch_filtered = (
            self.cfg.filter_alpha * ankle_pitch_raw +
            (1.0 - self.cfg.filter_alpha) * self.ankle_pitch_filtered
        )
        self.ankle_roll_filtered = (
            self.cfg.filter_alpha * ankle_roll_raw +
            (1.0 - self.cfg.filter_alpha) * self.ankle_roll_filtered
        )
        self.hip_pitch_filtered = (
            self.cfg.filter_alpha * hip_pitch_raw +
            (1.0 - self.cfg.filter_alpha) * self.hip_pitch_filtered
        )
        self.hip_roll_filtered = (
            self.cfg.filter_alpha * hip_roll_raw +
            (1.0 - self.cfg.filter_alpha) * self.hip_roll_filtered
        )

        targets['left_hip_pitch_joint'] = targets.get('left_hip_pitch_joint', 0.0) + self.hip_pitch_filtered
        targets['right_hip_pitch_joint'] = targets.get('right_hip_pitch_joint', 0.0) + self.hip_pitch_filtered
        targets['left_hip_roll_joint'] = targets.get('left_hip_roll_joint', 0.0) + self.hip_roll_filtered
        targets['right_hip_roll_joint'] = targets.get('right_hip_roll_joint', 0.0) - self.hip_roll_filtered

        targets['left_ankle_pitch_joint'] = targets.get('left_ankle_pitch_joint', 0.0) + self.ankle_pitch_filtered
        targets['right_ankle_pitch_joint'] = targets.get('right_ankle_pitch_joint', 0.0) + self.ankle_pitch_filtered
        targets['left_ankle_roll_joint'] = self.ankle_roll_filtered
        targets['right_ankle_roll_joint'] = -self.ankle_roll_filtered

        return targets

    def _tracking_error(self, targets: Dict[str, float], state: RobotState) -> float:
        joints = [
            'left_hip_pitch_joint',
            'left_hip_roll_joint',
            'left_hip_yaw_joint',
            'left_knee_joint',
            'left_ankle_pitch_joint',
            'left_ankle_roll_joint',
            'right_hip_pitch_joint',
            'right_hip_roll_joint',
            'right_hip_yaw_joint',
            'right_knee_joint',
            'right_ankle_pitch_joint',
            'right_ankle_roll_joint',
            'waist_yaw_joint',
        ]
        errors = []
        for joint in joints:
            if joint in targets and joint in state.joint_positions:
                errors.append(abs(targets[joint] - state.joint_positions[joint]))
        return float(max(errors)) if errors else 0.0

    def _apply_filter(self, targets: Dict[str, float]) -> Dict[str, float]:
        filtered = {}
        for joint, target in targets.items():
            if joint not in self.filtered_commands:
                self.filtered_commands[joint] = target
            else:
                self.filtered_commands[joint] = (
                    self.cfg.filter_alpha * target +
                    (1.0 - self.cfg.filter_alpha) * self.filtered_commands[joint]
                )
            filtered[joint] = self.filtered_commands[joint]
        return filtered

    def _control_loop(self):
        if not self.timer_started:
            return

        state = self._build_state()
        if state is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = now
        if self.last_control_time is None:
            self.last_control_time = now
        dt = max(0.0, now - self.last_control_time)
        self.last_control_time = now

        elapsed = now - self.start_time - self.phase_time_offset
        if self.cfg.walking_enabled:
            phase, progress = self._phase_progress(elapsed)
        else:
            if not self.cfg.balance_active:
                return
            phase, progress = Phase.STABILIZE, 0.0

        targets = self._build_targets(state, phase, progress)
        if self.cfg.hold_enabled and self.cfg.balance_active and not self.cfg.walking_enabled:
            error_mag = float(np.hypot(self.last_com_error_x, self.last_com_error_y))
            vel_mag = float(np.hypot(self.last_com_vel_x, self.last_com_vel_y))
            stable = (
                error_mag <= self.cfg.hold_com_threshold and
                vel_mag <= self.cfg.hold_vel_threshold
            )
            if stable:
                if self.stable_since is None:
                    self.stable_since = now
                elif not self.hold_active and (now - self.stable_since) >= self.cfg.hold_enter_duration:
                    self.hold_active = True
                    self.hold_targets = dict(self.filtered_commands or targets)
                    self.get_logger().info('Holding balance pose (stable).')
            else:
                self.stable_since = None

            if self.hold_active:
                release = (
                    error_mag >= self.cfg.hold_release_com_threshold or
                    vel_mag >= self.cfg.hold_release_vel_threshold
                )
                if release:
                    self.hold_active = False
                    self.hold_targets = {}
                    self.get_logger().info('Released hold (error increased).')
                else:
                    targets = dict(self.hold_targets)
        elif self.cfg.walking_enabled and self.cfg.phase_hold_enabled:
            tracking_error = self._tracking_error(targets, state)
            if (
                progress >= self.cfg.phase_hold_progress_gate and
                tracking_error > self.cfg.phase_error_threshold and
                self.phase_hold_accum < self.cfg.phase_hold_max
            ):
                self.phase_time_offset += dt
                self.phase_hold_accum += dt
            else:
                self.phase_hold_accum = 0.0
        else:
            self.stable_since = None
            self.hold_active = False
            self.hold_targets = {}
        targets = self._apply_filter(targets)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(targets.keys())
        msg.position = [targets[name] for name in msg.name]
        self.target_pub.publish(msg)
        self._publish_balance_markers(state)

        if self.cfg.log_period > 0.0:
            now = self.get_clock().now().nanoseconds / 1e9
            if self.last_log_time is None or (now - self.last_log_time) >= self.cfg.log_period:
                self.last_log_time = now
                com_y = state.com[1] if state.com is not None else 0.0
                err_mag = float(np.hypot(self.last_com_error_x, self.last_com_error_y))
                vel_mag = float(np.hypot(self.last_com_vel_x, self.last_com_vel_y))
                tracking_err = self._tracking_error(self.filtered_commands, state)
                self.get_logger().info(
                    f'Phase={phase.name} progress={progress:.2f} com_y={com_y:.3f} '
                    f'err={err_mag:.3f} vel={vel_mag:.3f} '
                    f'track={tracking_err:.3f} hold={self.hold_active}'
                )

    def _publish_balance_markers(self, state: RobotState):
        if state.com is None:
            return

        now = self.get_clock().now().to_msg()
        com_msg = PointStamped()
        com_msg.header.stamp = now
        com_msg.header.frame_id = 'world'
        com_point = np.array(state.com, dtype=float)
        if not self.cfg.com_in_world and self.base_pose_msg is not None:
            pose = self.base_pose_msg.pose
            rot = self._quat_to_rot(pose.orientation)
            com_point = rot @ com_point + np.array(
                [pose.position.x, pose.position.y, pose.position.z], dtype=float
            )
        com_msg.point.x = float(com_point[0])
        com_msg.point.y = float(com_point[1])
        com_msg.point.z = float(com_point[2])
        self.com_pub.publish(com_msg)

        zmp_msg = PointStamped()
        zmp_msg.header.stamp = now
        zmp_msg.header.frame_id = 'world'
        zmp_msg.point.x = float(com_point[0])
        zmp_msg.point.y = float(com_point[1])
        zmp_msg.point.z = 0.0
        self.zmp_pub.publish(zmp_msg)

        markers = MarkerArray()
        com_marker = Marker()
        com_marker.header = com_msg.header
        com_marker.ns = 'com'
        com_marker.id = 0
        com_marker.type = Marker.SPHERE
        com_marker.action = Marker.ADD
        com_marker.pose.position.x = com_msg.point.x
        com_marker.pose.position.y = com_msg.point.y
        com_marker.pose.position.z = com_msg.point.z
        com_marker.pose.orientation.w = 1.0
        com_marker.scale.x = 0.04
        com_marker.scale.y = 0.04
        com_marker.scale.z = 0.04
        com_marker.color.r = 0.1
        com_marker.color.g = 0.9
        com_marker.color.b = 0.1
        com_marker.color.a = 0.9

        zmp_marker = Marker()
        zmp_marker.header = zmp_msg.header
        zmp_marker.ns = 'zmp'
        zmp_marker.id = 1
        zmp_marker.type = Marker.SPHERE
        zmp_marker.action = Marker.ADD
        zmp_marker.pose.position.x = zmp_msg.point.x
        zmp_marker.pose.position.y = zmp_msg.point.y
        zmp_marker.pose.position.z = zmp_msg.point.z
        zmp_marker.pose.orientation.w = 1.0
        zmp_marker.scale.x = 0.05
        zmp_marker.scale.y = 0.05
        zmp_marker.scale.z = 0.05
        zmp_marker.color.r = 0.9
        zmp_marker.color.g = 0.1
        zmp_marker.color.b = 0.1
        zmp_marker.color.a = 0.9

        markers.markers.append(com_marker)
        markers.markers.append(zmp_marker)
        self.marker_pub.publish(markers)

    def _joint_state_callback(self, msg: JointState):
        self.joint_state = msg
        if not self.timer_started:
            self.timer_started = True
            self.timer = self.create_timer(1.0 / self.cfg.control_rate, self._control_loop)
            self.get_logger().info('Starting control loop')


def main(args=None):
    rclpy.init(args=args)
    node = WbcPinocchioController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
