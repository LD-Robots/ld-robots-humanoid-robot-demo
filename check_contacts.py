#!/usr/bin/env python3
"""
Diagnostic script to check contact sensors and foot positions
"""
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import sys


class ContactDiagnostic(Node):
    def __init__(self):
        super().__init__('contact_diagnostic')

        # Subscribe to contact sensors
        self.left_contact_sub = self.create_subscription(
            Contacts,
            '/left_foot/contact',
            self.left_contact_callback,
            10
        )

        self.right_contact_sub = self.create_subscription(
            Contacts,
            '/right_foot/contact',
            self.right_contact_callback,
            10
        )

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.left_contact_count = 0
        self.right_contact_count = 0
        self.joint_states_received = False

        # Timer to print status
        self.timer = self.create_timer(1.0, self.print_status)

        self.get_logger().info('Contact diagnostic node started')
        self.get_logger().info('Waiting for contact sensor data...')

    def left_contact_callback(self, msg):
        self.left_contact_count += 1
        if len(msg.contacts) > 0:
            self.get_logger().info(f'LEFT FOOT CONTACT DETECTED! ({len(msg.contacts)} contacts)')
            for i, contact in enumerate(msg.contacts):
                self.get_logger().info(f'  Contact {i}: {len(contact.positions)} positions')

    def right_contact_callback(self, msg):
        self.right_contact_count += 1
        if len(msg.contacts) > 0:
            self.get_logger().info(f'RIGHT FOOT CONTACT DETECTED! ({len(msg.contacts)} contacts)')
            for i, contact in enumerate(msg.contacts):
                self.get_logger().info(f'  Contact {i}: {len(contact.positions)} positions')

    def joint_state_callback(self, msg):
        self.joint_states_received = True

    def print_status(self):
        self.get_logger().info(f'Status - Left: {self.left_contact_count} msgs, Right: {self.right_contact_count} msgs, Joints: {self.joint_states_received}')


def main(args=None):
    rclpy.init(args=args)
    node = ContactDiagnostic()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
