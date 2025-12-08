#!/usr/bin/env python3
"""
Gripper controller node for the humanoid robot.
Controls gripper opening/closing for object grasping.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64


class GripperController(Node):
    """Control gripper for grasping operations."""

    def __init__(self):
        super().__init__('gripper_controller')

        self.declare_parameter('gripper_side', 'left')  # 'left' or 'right'
        self.declare_parameter('max_position', 0.04)  # meters
        self.declare_parameter('min_position', 0.0)  # meters

        self.gripper_side = self.get_parameter('gripper_side').value
        self.max_position = self.get_parameter('max_position').value
        self.min_position = self.get_parameter('min_position').value

        # Action client for gripper control
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{self.gripper_side}_gripper_controller/follow_joint_trajectory'
        )

        # Subscribe to gripper commands (0.0 = closed, 1.0 = open)
        self.gripper_cmd_sub = self.create_subscription(
            Float64,
            f'/{self.gripper_side}_gripper/command',
            self.gripper_command_callback,
            10
        )

        self.get_logger().info(f'{self.gripper_side.capitalize()} gripper controller started')

    def gripper_command_callback(self, msg: Float64):
        """
        Receive gripper position command.

        Args:
            msg: Float64 with value 0.0 (closed) to 1.0 (open)
        """
        # Clamp command to valid range
        command = max(0.0, min(1.0, msg.data))

        # Convert to actual gripper position
        target_position = self.min_position + command * (self.max_position - self.min_position)

        # Send trajectory to gripper
        self.move_gripper(target_position)

    def move_gripper(self, position: float):
        """
        Move gripper to target position.

        Args:
            position: Target position in meters
        """
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Gripper action server not available')
            return

        # Create trajectory
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = [f'{self.gripper_side}_gripper']

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.5 * 1e9)  # 500ms

        goal_msg.trajectory.points.append(point)
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle action goal response."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Gripper goal rejected')
            return

        self.get_logger().debug('Gripper goal accepted')


def main(args=None):
    rclpy.init(args=args)

    # Create controllers for both grippers
    left_gripper = GripperController()
    left_gripper.get_parameter('gripper_side').value = 'left'

    try:
        rclpy.spin(left_gripper)
    except KeyboardInterrupt:
        pass
    finally:
        left_gripper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
