#!/usr/bin/env python3
"""
Position Controller for MuJoCo Humanoid Robot.
Sends target positions - MuJoCo handles PD control internally.
"""

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class MuJoCoController(Node):
    """Simple position controller - MuJoCo does PD internally."""

    def __init__(self):
        super().__init__('mujoco_controller')

        self.declare_parameter('initial_pose_yaml', '')
        self.declare_parameter('initial_pose_key', '')
        self.initial_pose_yaml = self.get_parameter('initial_pose_yaml').value
        self.initial_pose_key = self.get_parameter('initial_pose_key').value

        # Actuator order (must match MuJoCo model actuator order)
        self.actuator_order = [
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
            'left_shoulder_pitch_joint',
            'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint',
            'left_elbow_joint',
            'left_wrist_roll_joint',
            'left_wrist_pitch_joint',
            'left_wrist_yaw_joint',
            'right_shoulder_pitch_joint',
            'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint',
            'right_elbow_joint',
            'right_wrist_roll_joint',
            'right_wrist_pitch_joint',
            'right_wrist_yaw_joint',
        ]

        # Default target positions for stable standing (in radians)
        self.target_positions = {
            # Legs - slight knee bend for stability
            'left_hip_pitch_joint': 0.0,
            'left_hip_roll_joint': 0.0,
            'left_hip_yaw_joint': 0.0,
            'left_knee_joint': 0.1,  # Slight bend
            'left_ankle_pitch_joint': -0.05,  # Compensate for knee bend
            'left_ankle_roll_joint': 0.0,
            'right_hip_pitch_joint': 0.0,
            'right_hip_roll_joint': 0.0,
            'right_hip_yaw_joint': 0.0,
            'right_knee_joint': 0.1,  # Slight bend
            'right_ankle_pitch_joint': -0.05,  # Compensate for knee bend
            'right_ankle_roll_joint': 0.0,
            # Waist
            'waist_yaw_joint': 0.0,
            # Arms - down by sides
            'left_shoulder_pitch_joint': 0.0,
            'left_shoulder_roll_joint': 0.0,
            'left_shoulder_yaw_joint': 0.0,
            'left_elbow_joint': 0.0,
            'left_wrist_roll_joint': 0.0,
            'left_wrist_pitch_joint': 0.0,
            'left_wrist_yaw_joint': 0.0,
            'right_shoulder_pitch_joint': 0.0,
            'right_shoulder_roll_joint': 0.0,
            'right_shoulder_yaw_joint': 0.0,
            'right_elbow_joint': 0.0,
            'right_wrist_roll_joint': 0.0,
            'right_wrist_pitch_joint': 0.0,
            'right_wrist_yaw_joint': 0.0,
        }

        loaded_pose = self._load_initial_pose()
        if loaded_pose:
            for joint_name, value in loaded_pose.items():
                if joint_name in self.target_positions:
                    self.target_positions[joint_name] = value
            self.get_logger().info(
                f'Loaded initial pose for {len(loaded_pose)} joints from YAML'
            )

        # Subscribe to target position updates
        self.target_sub = self.create_subscription(
            JointState,
            'target_positions',
            self.target_callback,
            10
        )

        # ROS 2 Publishers - send positions to MuJoCo
        self.position_cmd_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)

        self.get_logger().info('MuJoCo Position Controller initialized')
        self.get_logger().info('Sending target positions - MuJoCo handles PD control')

    def _load_initial_pose(self):
        if not self.initial_pose_yaml:
            return {}
        try:
            with open(self.initial_pose_yaml, 'r') as handle:
                data = yaml.safe_load(handle) or {}
        except Exception as exc:
            self.get_logger().warn(f'Failed to load initial pose YAML: {exc}')
            return {}

        joint_positions = {}
        if isinstance(data, dict):
            if self.initial_pose_key and isinstance(data.get(self.initial_pose_key), dict):
                joint_positions = data.get(self.initial_pose_key, {}).get('joint_positions', {})
            elif isinstance(data.get('joint_positions'), dict):
                joint_positions = data.get('joint_positions', {})
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
                    pose[joint_name] = float(group_vals[short_name])
        return pose

    def target_callback(self, msg):
        """Update target positions from user commands."""
        for i, name in enumerate(msg.name):
            if name in self.target_positions and i < len(msg.position):
                self.target_positions[name] = msg.position[i]

    def publish_commands(self):
        """Publish target positions in actuator order."""
        # Build position commands in actuator order
        positions = [self.target_positions[name] for name in self.actuator_order]

        # Publish
        cmd_msg = Float64MultiArray()
        cmd_msg.data = positions
        self.position_cmd_pub.publish(cmd_msg)


def main(args=None):
    import time
    rclpy.init(args=args)
    controller = MuJoCoController()

    # Manual control loop at 100 Hz (timers don't work reliably)
    rate_hz = 100.0
    period = 1.0 / rate_hz

    controller.get_logger().info(f'Starting control loop at {rate_hz} Hz')

    try:
        while rclpy.ok():
            loop_start = time.time()

            # Process callbacks
            rclpy.spin_once(controller, timeout_sec=0.0)

            # Publish commands
            controller.publish_commands()

            # Maintain rate
            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
