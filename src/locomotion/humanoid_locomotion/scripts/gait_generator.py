#!/usr/bin/env python3
"""
Gait generator node for the humanoid robot.
Generates walking patterns and trajectories for bipedal locomotion.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import numpy as np
from typing import List


class GaitGenerator(Node):
    """Generate walking gaits for the humanoid robot."""

    def __init__(self):
        super().__init__('gait_generator')

        self.declare_parameter('step_length', 0.1)  # meters
        self.declare_parameter('step_height', 0.05)  # meters
        self.declare_parameter('step_duration', 0.8)  # seconds
        self.declare_parameter('double_support_ratio', 0.2)  # 0-1

        self.step_length = self.get_parameter('step_length').value
        self.step_height = self.get_parameter('step_height').value
        self.step_duration = self.get_parameter('step_duration').value
        self.double_support_ratio = self.get_parameter('double_support_ratio').value

        # Current velocity command
        self.cmd_vel = Twist()

        # Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers for leg trajectories
        self.left_leg_pub = self.create_publisher(
            JointTrajectory,
            '/left_leg_controller/joint_trajectory',
            10
        )

        self.right_leg_pub = self.create_publisher(
            JointTrajectory,
            '/right_leg_controller/joint_trajectory',
            10
        )

        # Gait generation timer
        self.gait_timer = self.create_timer(
            self.step_duration,
            self.generate_step
        )

        self.is_left_foot_swing = False

        self.get_logger().info('Gait generator node started')

    def cmd_vel_callback(self, msg: Twist):
        """Receive velocity commands."""
        self.cmd_vel = msg

    def generate_step(self):
        """Generate a single step of the walking gait."""
        # Check if we should be walking
        if abs(self.cmd_vel.linear.x) < 0.01 and abs(self.cmd_vel.linear.y) < 0.01:
            # Not walking, maintain stance
            return

        # Generate trajectories for swing and stance legs
        if self.is_left_foot_swing:
            swing_traj = self.generate_swing_trajectory('left')
            stance_traj = self.generate_stance_trajectory('right')

            self.left_leg_pub.publish(swing_traj)
            self.right_leg_pub.publish(stance_traj)
        else:
            swing_traj = self.generate_swing_trajectory('right')
            stance_traj = self.generate_stance_trajectory('left')

            self.right_leg_pub.publish(swing_traj)
            self.left_leg_pub.publish(stance_traj)

        # Alternate swing leg
        self.is_left_foot_swing = not self.is_left_foot_swing

        self.get_logger().debug(f'Generated step, swing leg: {"left" if not self.is_left_foot_swing else "right"}')

    def generate_swing_trajectory(self, leg: str) -> JointTrajectory:
        """
        Generate swing phase trajectory for a leg.

        Args:
            leg: 'left' or 'right'

        Returns:
            JointTrajectory message
        """
        trajectory = JointTrajectory()

        # Define joint names
        joints = [
            f'{leg}_hip_yaw',
            f'{leg}_hip_roll',
            f'{leg}_hip_pitch',
            f'{leg}_knee_pitch',
            f'{leg}_ankle_pitch',
            f'{leg}_ankle_roll'
        ]
        trajectory.joint_names = joints

        # Generate trajectory points
        num_points = 10
        for i in range(num_points):
            point = JointTrajectoryPoint()

            # Time from start
            t = (i + 1) / num_points * self.step_duration

            # Simple sinusoidal swing trajectory (placeholder)
            phase = (i + 1) / num_points * np.pi

            # Hip pitch for forward swing
            hip_pitch = 0.3 * np.sin(phase)

            # Knee pitch (bend during swing)
            knee_pitch = -0.6 * np.sin(phase)

            # Ankle pitch
            ankle_pitch = 0.3 * np.sin(phase)

            point.positions = [
                0.0,  # hip yaw
                0.0,  # hip roll
                hip_pitch,
                knee_pitch,
                ankle_pitch,
                0.0   # ankle roll
            ]

            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            trajectory.points.append(point)

        trajectory.header.stamp = self.get_clock().now().to_msg()

        return trajectory

    def generate_stance_trajectory(self, leg: str) -> JointTrajectory:
        """
        Generate stance phase trajectory for a leg.

        Args:
            leg: 'left' or 'right'

        Returns:
            JointTrajectory message
        """
        trajectory = JointTrajectory()

        # Define joint names
        joints = [
            f'{leg}_hip_yaw',
            f'{leg}_hip_roll',
            f'{leg}_hip_pitch',
            f'{leg}_knee_pitch',
            f'{leg}_ankle_pitch',
            f'{leg}_ankle_roll'
        ]
        trajectory.joint_names = joints

        # Generate trajectory points
        num_points = 10
        for i in range(num_points):
            point = JointTrajectoryPoint()

            # Time from start
            t = (i + 1) / num_points * self.step_duration

            # Stance leg remains more stable (placeholder)
            point.positions = [
                0.0,   # hip yaw
                0.0,   # hip roll
                0.0,   # hip pitch
                0.0,   # knee pitch
                0.0,   # ankle pitch
                0.0    # ankle roll
            ]

            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            trajectory.points.append(point)

        trajectory.header.stamp = self.get_clock().now().to_msg()

        return trajectory


def main(args=None):
    rclpy.init(args=args)
    node = GaitGenerator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
