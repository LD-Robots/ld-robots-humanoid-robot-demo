#!/usr/bin/env python3
"""
Simplified Pressure Mat Visualizer - Uses only estimations, no bridge needed

This version creates a heatmap based on simple heuristics without
relying on the problematic contact sensor bridge.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2


class SimplePressureMatVisualizer(Node):
    """Simplified pressure visualizer - shows static demo pattern."""

    def __init__(self):
        super().__init__('pressure_mat_visualizer')

        # Parameters
        self.declare_parameter('grid_size_x', 20)
        self.declare_parameter('grid_size_y', 12)
        self.declare_parameter('update_rate', 30.0)

        self.grid_size_x = self.get_parameter('grid_size_x').value
        self.grid_size_y = self.get_parameter('grid_size_y').value

        # Initialize pressure grid
        self.pressure_grid = np.zeros((self.grid_size_y, self.grid_size_x), dtype=np.float32)

        # Create demo pressure points (simulating feet)
        self._add_demo_pressure()

        # Publisher
        self.heatmap_pub = self.create_publisher(Image, '/pressure_mat/heatmap', 10)

        # Timer
        update_period = 1.0 / self.get_parameter('update_rate').value
        self.timer = self.create_timer(update_period, self.publish_visualization)

        self.get_logger().info('Simple Pressure Mat Visualizer started (DEMO MODE)')
        self.get_logger().info(f'Grid size: {self.grid_size_x}x{self.grid_size_y}')

    def _add_demo_pressure(self):
        """Add demo pressure points to simulate foot contact."""
        # Left foot (around grid position 8, 4)
        self.pressure_grid[3:6, 7:10] = 0.8

        # Right foot (around grid position 8, 8)
        self.pressure_grid[7:10, 7:10] = 0.8

        # Add some variation
        self.pressure_grid += np.random.random((self.grid_size_y, self.grid_size_x)) * 0.1

    def publish_visualization(self):
        """Publish heatmap image."""
        # Normalize to 0-255
        if np.max(self.pressure_grid) > 0:
            normalized = (self.pressure_grid / np.max(self.pressure_grid) * 255).astype(np.uint8)
        else:
            normalized = np.zeros_like(self.pressure_grid, dtype=np.uint8)

        # Apply colormap
        heatmap = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)

        # Resize
        display_size = (400, 240)
        heatmap_resized = cv2.resize(heatmap, display_size, interpolation=cv2.INTER_LINEAR)

        # Add grid lines
        cell_width = display_size[0] // self.grid_size_x
        cell_height = display_size[1] // self.grid_size_y

        for i in range(1, self.grid_size_x):
            x = i * cell_width
            cv2.line(heatmap_resized, (x, 0), (x, display_size[1]), (50, 50, 50), 1)

        for i in range(1, self.grid_size_y):
            y = i * cell_height
            cv2.line(heatmap_resized, (0, y), (display_size[0], y), (50, 50, 50), 1)

        # Add text
        max_pressure = np.max(self.pressure_grid)
        text = f'Max: {max_pressure:.2f}N (DEMO)'
        cv2.putText(heatmap_resized, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)

        # Convert to ROS message
        try:
            image_msg = Image()
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'pressure_mat'
            image_msg.height = heatmap_resized.shape[0]
            image_msg.width = heatmap_resized.shape[1]
            image_msg.encoding = 'bgr8'
            image_msg.is_bigendian = 0
            image_msg.step = heatmap_resized.shape[1] * 3
            image_msg.data = heatmap_resized.tobytes()

            self.heatmap_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SimplePressureMatVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
