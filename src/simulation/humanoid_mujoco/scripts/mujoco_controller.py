#!/usr/bin/env python3
"""
Position Controller for MuJoCo Humanoid Robot.
Sends target positions - MuJoCo handles PD control internally.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class MuJoCoController(Node):
    """Simple position controller - MuJoCo does PD internally."""

    def __init__(self):
        super().__init__('mujoco_controller')

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

        # Target positions for stable standing (in radians)
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
