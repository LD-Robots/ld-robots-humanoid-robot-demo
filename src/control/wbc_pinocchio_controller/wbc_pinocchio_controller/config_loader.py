"""Load and validate WBC controller parameters."""

from dataclasses import dataclass
from typing import Any


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _as_float(value: Any) -> float:
    try:
        return float(value)
    except Exception:
        return 0.0


def _as_str(value: Any) -> str:
    if value is None:
        return ''
    return str(value)


@dataclass
class WbcConfig:
    model_xml_path: str = ''
    urdf_path: str = ''
    target_positions_topic: str = '/target_positions'
    joint_states_topic: str = '/joint_states'
    imu_topic: str = '/imu/data'
    base_pose_topic: str = '/torso/pose'
    com_in_world: bool = False
    hold_enabled: bool = True
    hold_enter_duration: float = 0.5
    hold_com_threshold: float = 0.005
    hold_vel_threshold: float = 0.02
    hold_release_com_threshold: float = 0.015
    hold_release_vel_threshold: float = 0.05
    phase_hold_enabled: bool = True
    phase_error_threshold: float = 0.06
    phase_hold_max: float = 1.5
    phase_hold_progress_gate: float = 0.9
    control_rate: float = 100.0
    walking_enabled: bool = True
    balance_active: bool = True
    stabilize_duration: float = 3.0
    step_duration: float = 4.0
    step_length: float = 0.04
    step_height: float = 0.07
    stance_width: float = 0.16
    support_knee_bend: float = 0.05
    swing_knee_bend: float = 0.7
    support_hip_pitch: float = 0.15
    swing_hip_pitch: float = 0.9
    support_ankle_pitch: float = 0.0
    swing_ankle_pitch: float = 0.25
    hip_roll_shift: float = 0.05
    swing_hip_roll_out: float = 0.0
    support_hip_yaw: float = 0.0
    swing_hip_yaw: float = 0.0
    balance_kp_pitch: float = 1.5
    balance_kd_pitch: float = 0.4
    balance_kp_roll: float = 1.5
    balance_kd_roll: float = 0.4
    balance_pitch_sign: float = 1.0
    imu_kp_pitch: float = 0.0
    imu_kd_pitch: float = 0.0
    imu_kp_roll: float = 0.0
    imu_kd_roll: float = 0.0
    ankle_pitch_limit: float = 0.12
    hip_pitch_limit: float = 0.12
    ankle_roll_limit: float = 0.12
    hip_roll_limit: float = 0.12
    com_deadzone: float = 0.005
    foot_length: float = 0.12
    foot_width: float = 0.08
    prediction_time: float = 0.2
    filter_alpha: float = 0.3
    use_crocoddyl: bool = False
    log_period: float = 1.0
    initial_pose_yaml: str = ''
    initial_pose_key: str = ''
    left_foot_frame: str = 'left_ankle_roll_link'
    right_foot_frame: str = 'right_ankle_roll_link'
    support_center_x_offset: float = 0.0


def load_wbc_config(node) -> WbcConfig:
    defaults = WbcConfig()

    node.declare_parameter('model_xml_path', defaults.model_xml_path)
    node.declare_parameter('urdf_path', defaults.urdf_path)
    node.declare_parameter('target_positions_topic', defaults.target_positions_topic)
    node.declare_parameter('joint_states_topic', defaults.joint_states_topic)
    node.declare_parameter('imu_topic', defaults.imu_topic)
    node.declare_parameter('base_pose_topic', defaults.base_pose_topic)
    node.declare_parameter('com_in_world', defaults.com_in_world)
    node.declare_parameter('hold_enabled', defaults.hold_enabled)
    node.declare_parameter('hold_enter_duration', defaults.hold_enter_duration)
    node.declare_parameter('hold_com_threshold', defaults.hold_com_threshold)
    node.declare_parameter('hold_vel_threshold', defaults.hold_vel_threshold)
    node.declare_parameter('hold_release_com_threshold', defaults.hold_release_com_threshold)
    node.declare_parameter('hold_release_vel_threshold', defaults.hold_release_vel_threshold)
    node.declare_parameter('phase_hold_enabled', defaults.phase_hold_enabled)
    node.declare_parameter('phase_error_threshold', defaults.phase_error_threshold)
    node.declare_parameter('phase_hold_max', defaults.phase_hold_max)
    node.declare_parameter('phase_hold_progress_gate', defaults.phase_hold_progress_gate)
    node.declare_parameter('control_rate', defaults.control_rate)
    node.declare_parameter('walking_enabled', defaults.walking_enabled)
    node.declare_parameter('balance_active', defaults.balance_active)
    node.declare_parameter('stabilize_duration', defaults.stabilize_duration)
    node.declare_parameter('step_duration', defaults.step_duration)
    node.declare_parameter('step_length', defaults.step_length)
    node.declare_parameter('step_height', defaults.step_height)
    node.declare_parameter('stance_width', defaults.stance_width)
    node.declare_parameter('support_knee_bend', defaults.support_knee_bend)
    node.declare_parameter('swing_knee_bend', defaults.swing_knee_bend)
    node.declare_parameter('support_hip_pitch', defaults.support_hip_pitch)
    node.declare_parameter('swing_hip_pitch', defaults.swing_hip_pitch)
    node.declare_parameter('support_ankle_pitch', defaults.support_ankle_pitch)
    node.declare_parameter('swing_ankle_pitch', defaults.swing_ankle_pitch)
    node.declare_parameter('hip_roll_shift', defaults.hip_roll_shift)
    node.declare_parameter('swing_hip_roll_out', defaults.swing_hip_roll_out)
    node.declare_parameter('support_hip_yaw', defaults.support_hip_yaw)
    node.declare_parameter('swing_hip_yaw', defaults.swing_hip_yaw)
    node.declare_parameter('balance_kp_pitch', defaults.balance_kp_pitch)
    node.declare_parameter('balance_kd_pitch', defaults.balance_kd_pitch)
    node.declare_parameter('balance_kp_roll', defaults.balance_kp_roll)
    node.declare_parameter('balance_kd_roll', defaults.balance_kd_roll)
    node.declare_parameter('balance_pitch_sign', defaults.balance_pitch_sign)
    node.declare_parameter('imu_kp_pitch', defaults.imu_kp_pitch)
    node.declare_parameter('imu_kd_pitch', defaults.imu_kd_pitch)
    node.declare_parameter('imu_kp_roll', defaults.imu_kp_roll)
    node.declare_parameter('imu_kd_roll', defaults.imu_kd_roll)
    node.declare_parameter('ankle_pitch_limit', defaults.ankle_pitch_limit)
    node.declare_parameter('hip_pitch_limit', defaults.hip_pitch_limit)
    node.declare_parameter('ankle_roll_limit', defaults.ankle_roll_limit)
    node.declare_parameter('hip_roll_limit', defaults.hip_roll_limit)
    node.declare_parameter('com_deadzone', defaults.com_deadzone)
    node.declare_parameter('foot_length', defaults.foot_length)
    node.declare_parameter('foot_width', defaults.foot_width)
    node.declare_parameter('prediction_time', defaults.prediction_time)
    node.declare_parameter('filter_alpha', defaults.filter_alpha)
    node.declare_parameter('use_crocoddyl', defaults.use_crocoddyl)
    node.declare_parameter('log_period', defaults.log_period)
    node.declare_parameter('initial_pose_yaml', defaults.initial_pose_yaml)
    node.declare_parameter('initial_pose_key', defaults.initial_pose_key)
    node.declare_parameter('left_foot_frame', defaults.left_foot_frame)
    node.declare_parameter('right_foot_frame', defaults.right_foot_frame)
    node.declare_parameter('support_center_x_offset', defaults.support_center_x_offset)

    return WbcConfig(
        model_xml_path=_as_str(node.get_parameter('model_xml_path').value),
        urdf_path=_as_str(node.get_parameter('urdf_path').value),
        target_positions_topic=_as_str(node.get_parameter('target_positions_topic').value),
        joint_states_topic=_as_str(node.get_parameter('joint_states_topic').value),
        imu_topic=_as_str(node.get_parameter('imu_topic').value),
        base_pose_topic=_as_str(node.get_parameter('base_pose_topic').value),
        com_in_world=_as_bool(node.get_parameter('com_in_world').value),
        hold_enabled=_as_bool(node.get_parameter('hold_enabled').value),
        hold_enter_duration=_as_float(node.get_parameter('hold_enter_duration').value),
        hold_com_threshold=_as_float(node.get_parameter('hold_com_threshold').value),
        hold_vel_threshold=_as_float(node.get_parameter('hold_vel_threshold').value),
        hold_release_com_threshold=_as_float(node.get_parameter('hold_release_com_threshold').value),
        hold_release_vel_threshold=_as_float(node.get_parameter('hold_release_vel_threshold').value),
        phase_hold_enabled=_as_bool(node.get_parameter('phase_hold_enabled').value),
        phase_error_threshold=_as_float(node.get_parameter('phase_error_threshold').value),
        phase_hold_max=_as_float(node.get_parameter('phase_hold_max').value),
        phase_hold_progress_gate=_as_float(node.get_parameter('phase_hold_progress_gate').value),
        control_rate=_as_float(node.get_parameter('control_rate').value),
        walking_enabled=_as_bool(node.get_parameter('walking_enabled').value),
        balance_active=_as_bool(node.get_parameter('balance_active').value),
        stabilize_duration=_as_float(node.get_parameter('stabilize_duration').value),
        step_duration=_as_float(node.get_parameter('step_duration').value),
        step_length=_as_float(node.get_parameter('step_length').value),
        step_height=_as_float(node.get_parameter('step_height').value),
        stance_width=_as_float(node.get_parameter('stance_width').value),
        support_knee_bend=_as_float(node.get_parameter('support_knee_bend').value),
        swing_knee_bend=_as_float(node.get_parameter('swing_knee_bend').value),
        support_hip_pitch=_as_float(node.get_parameter('support_hip_pitch').value),
        swing_hip_pitch=_as_float(node.get_parameter('swing_hip_pitch').value),
        support_ankle_pitch=_as_float(node.get_parameter('support_ankle_pitch').value),
        swing_ankle_pitch=_as_float(node.get_parameter('swing_ankle_pitch').value),
        hip_roll_shift=_as_float(node.get_parameter('hip_roll_shift').value),
        swing_hip_roll_out=_as_float(node.get_parameter('swing_hip_roll_out').value),
        support_hip_yaw=_as_float(node.get_parameter('support_hip_yaw').value),
        swing_hip_yaw=_as_float(node.get_parameter('swing_hip_yaw').value),
        balance_kp_pitch=_as_float(node.get_parameter('balance_kp_pitch').value),
        balance_kd_pitch=_as_float(node.get_parameter('balance_kd_pitch').value),
        balance_kp_roll=_as_float(node.get_parameter('balance_kp_roll').value),
        balance_kd_roll=_as_float(node.get_parameter('balance_kd_roll').value),
        balance_pitch_sign=_as_float(node.get_parameter('balance_pitch_sign').value),
        imu_kp_pitch=_as_float(node.get_parameter('imu_kp_pitch').value),
        imu_kd_pitch=_as_float(node.get_parameter('imu_kd_pitch').value),
        imu_kp_roll=_as_float(node.get_parameter('imu_kp_roll').value),
        imu_kd_roll=_as_float(node.get_parameter('imu_kd_roll').value),
        ankle_pitch_limit=_as_float(node.get_parameter('ankle_pitch_limit').value),
        hip_pitch_limit=_as_float(node.get_parameter('hip_pitch_limit').value),
        ankle_roll_limit=_as_float(node.get_parameter('ankle_roll_limit').value),
        hip_roll_limit=_as_float(node.get_parameter('hip_roll_limit').value),
        com_deadzone=_as_float(node.get_parameter('com_deadzone').value),
        foot_length=_as_float(node.get_parameter('foot_length').value),
        foot_width=_as_float(node.get_parameter('foot_width').value),
        prediction_time=_as_float(node.get_parameter('prediction_time').value),
        filter_alpha=_as_float(node.get_parameter('filter_alpha').value),
        use_crocoddyl=_as_bool(node.get_parameter('use_crocoddyl').value),
        log_period=_as_float(node.get_parameter('log_period').value),
        initial_pose_yaml=_as_str(node.get_parameter('initial_pose_yaml').value),
        initial_pose_key=_as_str(node.get_parameter('initial_pose_key').value),
        left_foot_frame=_as_str(node.get_parameter('left_foot_frame').value),
        right_foot_frame=_as_str(node.get_parameter('right_foot_frame').value),
        support_center_x_offset=_as_float(node.get_parameter('support_center_x_offset').value),
    )
