"""Balance correction logic for CoM-based stabilization."""

from dataclasses import dataclass
from typing import Optional

import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

from .state_estimator import RobotState


@dataclass
class BalanceCorrections:
    hip_pitch: float
    hip_roll: float
    ankle_pitch: float
    ankle_roll: float


class BalanceController:
    def __init__(
        self,
        node,
        *,
        com_in_world: bool,
        com_deadzone: float,
        foot_length: float,
        foot_width: float,
        support_center_x_offset: float,
        prediction_time: float,
        balance_kp_pitch: float,
        balance_kd_pitch: float,
        balance_kp_roll: float,
        balance_kd_roll: float,
        balance_pitch_sign: float,
        imu_kp_pitch: float,
        imu_kd_pitch: float,
        imu_kp_roll: float,
        imu_kd_roll: float,
        ankle_pitch_limit: float,
        ankle_roll_limit: float,
        hip_pitch_limit: float,
        hip_roll_limit: float,
        filter_alpha: float,
        log_period: float,
    ) -> None:
        self._node = node
        self._log_period = log_period
        self._last_log_time = None
        self._com_in_world = com_in_world
        self._com_deadzone = com_deadzone
        self._foot_length = foot_length
        self._foot_width = foot_width
        self._support_center_x_offset = support_center_x_offset
        self._prediction_time = prediction_time
        self._balance_kp_pitch = balance_kp_pitch
        self._balance_kd_pitch = balance_kd_pitch
        self._balance_kp_roll = balance_kp_roll
        self._balance_kd_roll = balance_kd_roll
        self._balance_pitch_sign = balance_pitch_sign
        self._imu_kp_pitch = imu_kp_pitch
        self._imu_kd_pitch = imu_kd_pitch
        self._imu_kp_roll = imu_kp_roll
        self._imu_kd_roll = imu_kd_roll
        self._ankle_pitch_limit = ankle_pitch_limit
        self._ankle_roll_limit = ankle_roll_limit
        self._hip_pitch_limit = hip_pitch_limit
        self._hip_roll_limit = hip_roll_limit
        self._filter_alpha = filter_alpha

        self._ankle_pitch_filtered = 0.0
        self._ankle_roll_filtered = 0.0
        self._hip_pitch_filtered = 0.0
        self._hip_roll_filtered = 0.0
        self.last_com_error_x = 0.0
        self.last_com_error_y = 0.0
        self.last_com_vel_x = 0.0
        self.last_com_vel_y = 0.0

    def compute(
        self,
        state: RobotState,
        base_pose_msg: Optional[PoseStamped],
        imu_msg: Optional[Imu],
    ) -> BalanceCorrections:
        com_error_x = 0.0
        com_error_y = 0.0
        com_vel_x = 0.0
        com_vel_y = 0.0
        support_center_x = 0.0
        support_center_y = 0.0

        if state.left_foot_pos is not None and state.right_foot_pos is not None:
            support_center = 0.5 * (state.left_foot_pos + state.right_foot_pos)
            support_center_x = float(support_center[0] + self._support_center_x_offset)
            support_center_y = float(support_center[1])
        elif self._com_in_world and base_pose_msg is not None:
            support_center_x = float(base_pose_msg.pose.position.x) + self._support_center_x_offset
            support_center_y = float(base_pose_msg.pose.position.y)

        if state.com is not None and state.com_vel is not None:
            com_error_x = state.com[0] - support_center_x
            com_error_y = state.com[1] - support_center_y
            com_vel_x = state.com_vel[0]
            com_vel_y = state.com_vel[1]

        self.last_com_error_x = com_error_x
        self.last_com_error_y = com_error_y
        self.last_com_vel_x = com_vel_x
        self.last_com_vel_y = com_vel_y

        if abs(com_error_x) < self._com_deadzone:
            com_error_x = 0.0
        if abs(com_error_y) < self._com_deadzone:
            com_error_y = 0.0

        support_limit_x = max(self._foot_length * 0.5, 1e-3)
        support_limit_y = max(self._foot_width * 0.5, 1e-3)
        com_margin_x = abs(com_error_x) / support_limit_x
        com_margin_y = abs(com_error_y) / support_limit_y

        com_predicted_x = com_error_x + com_vel_x * self._prediction_time
        com_predicted_y = com_error_y + com_vel_y * self._prediction_time
        com_error_blend_x = 0.7 * com_error_x + 0.3 * com_predicted_x
        com_error_blend_y = 0.7 * com_error_y + 0.3 * com_predicted_y

        correction_x = -(
            self._balance_kp_pitch * com_error_blend_x +
            self._balance_kd_pitch * com_vel_x
        ) * self._balance_pitch_sign
        correction_y = -(self._balance_kp_roll * com_error_blend_y + self._balance_kd_roll * com_vel_y)

        if imu_msg is not None and (self._imu_kp_pitch != 0.0 or self._imu_kp_roll != 0.0):
            roll, pitch = self._quat_to_rp(imu_msg.orientation)
            roll_rate = float(imu_msg.angular_velocity.x)
            pitch_rate = float(imu_msg.angular_velocity.y)
            correction_x -= self._imu_kp_pitch * pitch + self._imu_kd_pitch * pitch_rate
            correction_y -= self._imu_kp_roll * roll + self._imu_kd_roll * roll_rate

        com_tilt_mag = float(np.sqrt(com_margin_x**2 + com_margin_y**2))
        hip_ratio = float(np.tanh(com_tilt_mag / 0.3))
        ankle_ratio = 1.0 - 0.5 * hip_ratio

        ankle_pitch_raw = np.clip(
            -ankle_ratio * correction_x,
            -self._ankle_pitch_limit,
            self._ankle_pitch_limit,
        )
        ankle_roll_raw = np.clip(
            -ankle_ratio * correction_y,
            -self._ankle_roll_limit,
            self._ankle_roll_limit,
        )
        hip_pitch_raw = np.clip(
            -hip_ratio * correction_x,
            -self._hip_pitch_limit,
            self._hip_pitch_limit,
        )
        hip_roll_raw = np.clip(
            -hip_ratio * correction_y,
            -self._hip_roll_limit,
            self._hip_roll_limit,
        )

        self._ankle_pitch_filtered = (
            self._filter_alpha * ankle_pitch_raw +
            (1.0 - self._filter_alpha) * self._ankle_pitch_filtered
        )
        self._ankle_roll_filtered = (
            self._filter_alpha * ankle_roll_raw +
            (1.0 - self._filter_alpha) * self._ankle_roll_filtered
        )
        self._hip_pitch_filtered = (
            self._filter_alpha * hip_pitch_raw +
            (1.0 - self._filter_alpha) * self._hip_pitch_filtered
        )
        self._hip_roll_filtered = (
            self._filter_alpha * hip_roll_raw +
            (1.0 - self._filter_alpha) * self._hip_roll_filtered
        )

        if self._log_period > 0.0 and state.com is not None:
            now = self._node.get_clock().now().nanoseconds / 1e9
            if self._last_log_time is None or (now - self._last_log_time) >= self._log_period:
                self._last_log_time = now
                self._node.get_logger().info(
                    f'Balance debug: com_x={state.com[0]:.3f} '
                    f'support_x={support_center_x:.3f} err_x={com_error_x:.3f}'
                )

        return BalanceCorrections(
            hip_pitch=self._hip_pitch_filtered,
            hip_roll=self._hip_roll_filtered,
            ankle_pitch=self._ankle_pitch_filtered,
            ankle_roll=self._ankle_roll_filtered,
        )

    @staticmethod
    def _quat_to_rot(quat) -> np.ndarray:
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ])

    @staticmethod
    def _quat_to_rp(quat) -> tuple[float, float]:
        w, x, y, z = quat.w, quat.x, quat.y, quat.z
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = float(np.arctan2(sinr_cosp, cosr_cosp))
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = float(np.sign(sinp) * (np.pi / 2.0))
        else:
            pitch = float(np.arcsin(sinp))
        return roll, pitch
