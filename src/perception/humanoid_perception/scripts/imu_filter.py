#!/usr/bin/env python3
"""
IMU data filter node for the humanoid robot.
Filters and processes IMU data for better orientation estimation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np


class IMUFilter(Node):
    """Filter and process IMU data for orientation estimation."""

    def __init__(self):
        super().__init__('imu_filter')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('filtered_topic', '/imu/data_filtered')
        self.declare_parameter('filter_alpha', 0.9)

        imu_topic = self.get_parameter('imu_topic').value
        filtered_topic = self.get_parameter('filtered_topic').value
        self.alpha = self.get_parameter('filter_alpha').value

        # Previous values for filtering
        self.prev_angular_vel = np.zeros(3)
        self.prev_linear_acc = np.zeros(3)

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )

        # Publishers
        self.filtered_pub = self.create_publisher(
            Imu,
            filtered_topic,
            10
        )

        self.get_logger().info('IMU filter node started')

    def imu_callback(self, msg: Imu):
        """Process and filter IMU data."""
        try:
            # Apply low-pass filter to angular velocity
            angular_vel = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])

            filtered_angular_vel = (self.alpha * self.prev_angular_vel +
                                   (1 - self.alpha) * angular_vel)

            # Apply low-pass filter to linear acceleration
            linear_acc = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])

            filtered_linear_acc = (self.alpha * self.prev_linear_acc +
                                  (1 - self.alpha) * linear_acc)

            # Create filtered message
            filtered_msg = Imu()
            filtered_msg.header = msg.header
            filtered_msg.orientation = msg.orientation
            filtered_msg.orientation_covariance = msg.orientation_covariance

            filtered_msg.angular_velocity = Vector3(
                x=float(filtered_angular_vel[0]),
                y=float(filtered_angular_vel[1]),
                z=float(filtered_angular_vel[2])
            )
            filtered_msg.angular_velocity_covariance = msg.angular_velocity_covariance

            filtered_msg.linear_acceleration = Vector3(
                x=float(filtered_linear_acc[0]),
                y=float(filtered_linear_acc[1]),
                z=float(filtered_linear_acc[2])
            )
            filtered_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

            # Publish filtered data
            self.filtered_pub.publish(filtered_msg)

            # Update previous values
            self.prev_angular_vel = filtered_angular_vel
            self.prev_linear_acc = filtered_linear_acc

        except Exception as e:
            self.get_logger().error(f'Error filtering IMU data: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = IMUFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
