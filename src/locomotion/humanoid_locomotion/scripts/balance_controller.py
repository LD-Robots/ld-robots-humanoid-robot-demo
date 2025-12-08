#!/usr/bin/env python3
"""
Balance controller node for the humanoid robot.
Uses IMU feedback to maintain balance and upright posture.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np


class BalanceController(Node):
    """Maintain balance using IMU feedback."""

    def __init__(self):
        super().__init__('balance_controller')

        self.declare_parameter('kp_pitch', 0.5)
        self.declare_parameter('kp_roll', 0.5)
        self.declare_parameter('kd_pitch', 0.1)
        self.declare_parameter('kd_roll', 0.1)

        self.kp_pitch = self.get_parameter('kp_pitch').value
        self.kp_roll = self.get_parameter('kp_roll').value
        self.kd_pitch = self.get_parameter('kd_pitch').value
        self.kd_roll = self.get_parameter('kd_roll').value

        # Subscribe to IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publishers for balance corrections
        self.ankle_correction_pub = self.create_publisher(
            JointTrajectory,
            '/balance/ankle_correction',
            10
        )

        self.hip_correction_pub = self.create_publisher(
            JointTrajectory,
            '/balance/hip_correction',
            10
        )

        self.current_orientation = None
        self.prev_pitch = 0.0
        self.prev_roll = 0.0

        self.get_logger().info('Balance controller node started')

    def imu_callback(self, msg: Imu):
        """Process IMU data and generate balance corrections."""
        try:
            # Extract orientation (quaternion)
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            pitch, roll = self.quaternion_to_euler(qx, qy, qz, qw)

            # Calculate angular velocities
            pitch_vel = msg.angular_velocity.y
            roll_vel = msg.angular_velocity.x

            # PD control for balance
            pitch_correction = -(self.kp_pitch * pitch + self.kd_pitch * pitch_vel)
            roll_correction = -(self.kp_roll * roll + self.kd_roll * roll_vel)

            # Apply corrections to ankles
            self.apply_ankle_correction(pitch_correction, roll_correction)

            # Apply corrections to hips for larger disturbances
            if abs(pitch) > 0.2 or abs(roll) > 0.2:
                self.apply_hip_correction(pitch_correction, roll_correction)

        except Exception as e:
            self.get_logger().error(f'Error in balance control: {str(e)}')

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).

        Args:
            x, y, z, w: Quaternion components

        Returns:
            pitch, roll in radians
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        return pitch, roll

    def apply_ankle_correction(self, pitch_correction, roll_correction):
        """Apply balance corrections to ankle joints."""
        # Create trajectory for ankle corrections
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'left_ankle_pitch',
            'left_ankle_roll',
            'right_ankle_pitch',
            'right_ankle_roll'
        ]

        point = JointTrajectoryPoint()
        point.positions = [
            pitch_correction,
            roll_correction,
            pitch_correction,
            -roll_correction  # Opposite for right leg
        ]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.1 * 1e9)  # 100ms

        trajectory.points.append(point)
        trajectory.header.stamp = self.get_clock().now().to_msg()

        self.ankle_correction_pub.publish(trajectory)

    def apply_hip_correction(self, pitch_correction, roll_correction):
        """Apply balance corrections to hip joints for larger disturbances."""
        # Create trajectory for hip corrections
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'left_hip_pitch',
            'left_hip_roll',
            'right_hip_pitch',
            'right_hip_roll'
        ]

        point = JointTrajectoryPoint()
        point.positions = [
            pitch_correction * 0.5,  # Scale down for hips
            roll_correction * 0.5,
            pitch_correction * 0.5,
            -roll_correction * 0.5
        ]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.1 * 1e9)

        trajectory.points.append(point)
        trajectory.header.stamp = self.get_clock().now().to_msg()

        self.hip_correction_pub.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
