"""Utilities for loading joint mappings from YAML pose configs."""

from typing import Dict, Optional
import yaml


def load_initial_pose(
    yaml_path: str,
    pose_key: str,
    logger: Optional[object] = None,
) -> Dict[str, float]:
    if not yaml_path:
        return {}
    try:
        with open(yaml_path, 'r') as handle:
            data = yaml.safe_load(handle) or {}
    except Exception as exc:
        if logger is not None:
            logger.warn(f'Failed to load initial pose YAML: {exc}')
        return {}

    joint_positions = {}
    if isinstance(data, dict):
        if pose_key and isinstance(data.get(pose_key), dict):
            joint_positions = data.get(pose_key, {}).get('joint_positions', {})
        elif isinstance(data.get('joint_positions'), dict):
            joint_positions = data.get('joint_positions', {})
        elif isinstance(data.get('phase_1_standing'), dict):
            joint_positions = data.get('phase_1_standing', {}).get('joint_positions', {})
        else:
            for entry in data.values():
                if isinstance(entry, dict) and isinstance(entry.get('joint_positions'), dict):
                    joint_positions = entry.get('joint_positions', {})
                    break

    mapping = {
        'left_leg': {
            'hip_pitch': 'left_hip_pitch_joint',
            'hip_roll': 'left_hip_roll_joint',
            'hip_yaw': 'left_hip_yaw_joint',
            'knee': 'left_knee_joint',
            'ankle_pitch': 'left_ankle_pitch_joint',
            'ankle_roll': 'left_ankle_roll_joint',
        },
        'right_leg': {
            'hip_pitch': 'right_hip_pitch_joint',
            'hip_roll': 'right_hip_roll_joint',
            'hip_yaw': 'right_hip_yaw_joint',
            'knee': 'right_knee_joint',
            'ankle_pitch': 'right_ankle_pitch_joint',
            'ankle_roll': 'right_ankle_roll_joint',
        },
        'waist': {
            'yaw': 'waist_yaw_joint',
        },
        'left_arm': {
            'shoulder_pitch': 'left_shoulder_pitch_joint',
            'shoulder_roll': 'left_shoulder_roll_joint',
            'shoulder_yaw': 'left_shoulder_yaw_joint',
            'elbow': 'left_elbow_joint',
            'wrist_roll': 'left_wrist_roll_joint',
            'wrist_pitch': 'left_wrist_pitch_joint',
            'wrist_yaw': 'left_wrist_yaw_joint',
        },
        'right_arm': {
            'shoulder_pitch': 'right_shoulder_pitch_joint',
            'shoulder_roll': 'right_shoulder_roll_joint',
            'shoulder_yaw': 'right_shoulder_yaw_joint',
            'elbow': 'right_elbow_joint',
            'wrist_roll': 'right_wrist_roll_joint',
            'wrist_pitch': 'right_wrist_pitch_joint',
            'wrist_yaw': 'right_wrist_yaw_joint',
        },
    }

    pose = {}
    for group, joints in mapping.items():
        group_vals = joint_positions.get(group, {})
        if not isinstance(group_vals, dict):
            continue
        for short_name, joint_name in joints.items():
            if short_name in group_vals:
                try:
                    pose[joint_name] = float(group_vals[short_name])
                except (TypeError, ValueError):
                    if logger is not None:
                        logger.warn(f'Invalid value for {joint_name}: {group_vals[short_name]}')
    return pose
