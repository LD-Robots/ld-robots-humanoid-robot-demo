"""Balance correction logic for CoM-based stabilization."""

from dataclasses import dataclass
from typing import Optional

import numpy as np
from geometry_msgs.msg import PoseStamped

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
        *,
        com_in_world: bool,
        com_deadzone: float,
        foot_length: float,
        foot_width: float,
        prediction_time: float,
        balance_kp_pitch: float,
        balance_kd_pitch: float,
        balance_kp_roll: float,
        balance_kd_roll: float,
        ankle_pitch_limit: float,
        ankle_roll_limit: float,
        hip_pitch_limit: float,
        hip_roll_limit: float,
        filter_alpha: float,
    ) -> None:
        self._com_in_world = com_in_world
        self._com_deadzone = com_deadzone
        self._foot_length = foot_length
        self._foot_width = foot_width
        self._prediction_time = prediction_time
        self._balance_kp_pitch = balance_kp_pitch
        self._balance_kd_pitch = balance_kd_pitch
        self._balance_kp_roll = balance_kp_roll
        self._balance_kd_roll = balance_kd_roll
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
    ) -> BalanceCorrections:
        com_error_x = 0.0
        com_error_y = 0.0
        com_vel_x = 0.0
        com_vel_y = 0.0
        support_center_x = 0.0
        support_center_y = 0.0

        if self._com_in_world and base_pose_msg is not None:
            support_center_x = float(base_pose_msg.pose.position.x)
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

        correction_x = -(self._balance_kp_pitch * com_error_blend_x + self._balance_kd_pitch * com_vel_x)
        correction_y = -(self._balance_kp_roll * com_error_blend_y + self._balance_kd_roll * com_vel_y)

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

        return BalanceCorrections(
            hip_pitch=self._hip_pitch_filtered,
            hip_roll=self._hip_roll_filtered,
            ankle_pitch=self._ankle_pitch_filtered,
            ankle_roll=self._ankle_roll_filtered,
        )
