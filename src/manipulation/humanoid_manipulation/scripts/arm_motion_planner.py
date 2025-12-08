#!/usr/bin/env python3
"""
Arm motion planner node for the humanoid robot.
Provides high-level interface for arm motion planning and execution.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np


class ArmMotionPlanner(Node):
    """Plan and execute arm motions."""

    def __init__(self):
        super().__init__('arm_motion_planner')

        self.declare_parameter('arm_side', 'left')  # 'left' or 'right'
        self.arm_side = self.get_parameter('arm_side').value

        # Action client for arm control
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{self.arm_side}_arm_controller/follow_joint_trajectory'
        )

        # Subscribe to target pose commands
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            f'/{self.arm_side}_arm/target_pose',
            self.target_pose_callback,
            10
        )

        # Joint names for the arm
        self.arm_joints = [
            f'{self.arm_side}_shoulder_pitch',
            f'{self.arm_side}_shoulder_roll',
            f'{self.arm_side}_shoulder_yaw',
            f'{self.arm_side}_elbow_pitch',
            f'{self.arm_side}_wrist_pitch',
            f'{self.arm_side}_wrist_roll'
        ]

        self.get_logger().info(f'{self.arm_side.capitalize()} arm motion planner started')

    def target_pose_callback(self, msg: PoseStamped):
        """
        Receive target end-effector pose.

        Args:
            msg: Target pose in world frame
        """
        target_pose = msg.pose

        # Solve inverse kinematics (placeholder - use actual IK solver)
        joint_positions = self.solve_ik(target_pose)

        if joint_positions is not None:
            self.move_to_joint_positions(joint_positions)
        else:
            self.get_logger().warn('IK solution not found')

    def solve_ik(self, target_pose: Pose):
        """
        Solve inverse kinematics for target pose.

        Args:
            target_pose: Target end-effector pose

        Returns:
            List of joint positions or None if no solution
        """
        # Placeholder IK solver
        # In a real implementation, you would:
        # 1. Use KDL, TRAC-IK, or similar IK solver
        # 2. Or integrate with MoveIt2 for motion planning
        # 3. Consider joint limits and collision avoidance

        # Simple example: move to a predefined configuration
        # This should be replaced with actual IK
        joint_positions = [0.0, 0.5, 0.0, 1.0, 0.0, 0.0]

        self.get_logger().debug(f'IK solution: {joint_positions}')
        return joint_positions

    def move_to_joint_positions(self, joint_positions):
        """
        Move arm to target joint positions.

        Args:
            joint_positions: List of joint position values
        """
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Arm action server not available')
            return

        # Create trajectory
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = self.arm_joints

        # Create smooth trajectory with multiple waypoints
        num_waypoints = 5
        for i in range(1, num_waypoints + 1):
            point = JointTrajectoryPoint()

            # Interpolate from current to target
            # (In real use, get current positions from joint states)
            alpha = i / num_waypoints
            point.positions = [alpha * pos for pos in joint_positions]

            # Time for this waypoint
            t = i * 0.5  # 500ms per waypoint
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            goal_msg.trajectory.points.append(point)

        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle action goal response."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Arm motion goal rejected')
            return

        self.get_logger().info('Arm motion goal accepted')

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle action result."""
        result = future.result().result
        self.get_logger().info(f'Arm motion completed with error code: {result.error_code}')


def main(args=None):
    rclpy.init(args=args)
    node = ArmMotionPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
