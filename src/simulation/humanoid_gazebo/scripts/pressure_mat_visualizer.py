#!/usr/bin/env python3
"""
Pressure Mat Visualizer Node

This node subscribes to contact sensor data from the pressure mat in Gazebo
and creates a real-time heatmap visualization showing pressure distribution.

Topics:
    Subscribes to: /world/empty_world/model/pressure_mat/link/mat_base/sensor/pressure_mat_contact_sensor/contact
    Publishes to: /pressure_mat/heatmap (sensor_msgs/Image)
                  /pressure_mat/visualization_marker (visualization_msgs/MarkerArray)
"""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import cv2


class PressureMatVisualizer(Node):
    """Visualizes pressure distribution on a mat from contact sensor data."""

    def __init__(self):
        super().__init__('pressure_mat_visualizer')

        # Parameters
        self.declare_parameter('grid_size_x', 20)
        self.declare_parameter('grid_size_y', 12)  # 20x12 for 1.0m x 0.6m mat
        self.declare_parameter('mat_width', 1.0)   # meters
        self.declare_parameter('mat_height', 0.6)  # meters
        self.declare_parameter('update_rate', 30.0)  # Hz
        self.declare_parameter('force_scale', 100.0)  # Scale factor for force visualization
        self.declare_parameter('decay_rate', 0.9)  # How fast pressure decays (0-1)
        self.declare_parameter('smoothing_alpha', 0.3)  # Temporal smoothing (0-1, lower=more smooth)

        # Get parameters
        self.grid_size_x = self.get_parameter('grid_size_x').value
        self.grid_size_y = self.get_parameter('grid_size_y').value
        self.mat_width = self.get_parameter('mat_width').value
        self.mat_height = self.get_parameter('mat_height').value
        self.force_scale = self.get_parameter('force_scale').value
        self.decay_rate = self.get_parameter('decay_rate').value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value

        # Initialize pressure grid
        self.pressure_grid = np.zeros((self.grid_size_y, self.grid_size_x), dtype=np.float32)

        # Subscriber to contact sensor
        self.contact_sub = self.create_subscription(
            Contacts,
            '/world/empty_world/model/pressure_mat/link/mat_base/sensor/pressure_mat_contact_sensor/contact',
            self.contact_callback,
            10
        )

        # Publishers
        self.heatmap_pub = self.create_publisher(Image, '/pressure_mat/heatmap', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/pressure_mat/markers', 10)

        # Timer for visualization updates
        update_period = 1.0 / self.get_parameter('update_rate').value
        self.timer = self.create_timer(update_period, self.publish_visualization)

        self.get_logger().info('Pressure Mat Visualizer started')
        self.get_logger().info(f'Grid size: {self.grid_size_x}x{self.grid_size_y}')
        self.get_logger().info(f'Mat dimensions: {self.mat_width}m x {self.mat_height}m')

    def contact_callback(self, msg: Contacts):
        """Process contact sensor data and update pressure grid."""
        try:
            # Start fresh each frame
            new_grid = np.zeros((self.grid_size_y, self.grid_size_x), dtype=np.float32)

            # Process each contact
            for contact in msg.contacts:
                for position in contact.positions:
                    try:
                        # Convert to grid coordinates
                        x = float(position.x) + self.mat_width / 2.0
                        y = float(position.y) + self.mat_height / 2.0

                        if 0 <= x <= self.mat_width and 0 <= y <= self.mat_height:
                            grid_x = int((x / self.mat_width) * self.grid_size_x)
                            grid_y = int((y / self.mat_height) * self.grid_size_y)
                            grid_x = max(0, min(grid_x, self.grid_size_x - 1))
                            grid_y = max(0, min(grid_y, self.grid_size_y - 1))

                            # Mark contact and spread to neighbors with large radius for foot-shaped visualization
                            new_grid[grid_y, grid_x] = 1.0
                            self._spread_pressure_to_grid(new_grid, grid_x, grid_y, 1.0, radius=8)
                    except:
                        continue

            # Smooth blend: 20% new, 80% old (prevents flicker)
            self.pressure_grid = 0.2 * new_grid + 0.8 * self.pressure_grid

        except Exception as e:
            self.get_logger().error(f'Error in contact callback: {e}')

    def _spread_pressure_to_grid(self, grid, x, y, value, radius=1):
        """Spread pressure to neighboring cells for smoother visualization."""
        spread_factor = 0.3  # How much pressure spreads to neighbors

        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx == 0 and dy == 0:
                    continue

                nx = x + dx
                ny = y + dy

                if 0 <= nx < self.grid_size_x and 0 <= ny < self.grid_size_y:
                    distance = np.sqrt(dx**2 + dy**2)
                    spread_value = value * spread_factor / distance
                    grid[ny, nx] += spread_value

    def publish_visualization(self):
        """Publish heatmap image and 3D markers."""
        # Create heatmap image
        self._publish_heatmap()

        # Create 3D markers for RViz
        self._publish_markers()

    def _publish_heatmap(self):
        """Create and publish heatmap as an image."""
        # Zero out very small values (below threshold)
        threshold = 0.01
        self.pressure_grid[self.pressure_grid < threshold] = 0.0

        # Normalize pressure grid to 0-255 range
        if np.max(self.pressure_grid) > 0:
            normalized = (self.pressure_grid / np.max(self.pressure_grid) * 255).astype(np.uint8)
        else:
            normalized = np.zeros_like(self.pressure_grid, dtype=np.uint8)

        # Apply colormap (COLORMAP_JET: blue=low, red=high)
        heatmap = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)

        # Resize for better visibility
        display_size = (400, 240)  # Width x Height in pixels
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

        # Add text overlay with max pressure
        max_pressure = np.max(self.pressure_grid)
        text = f'Max: {max_pressure:.2f}N'
        cv2.putText(heatmap_resized, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)

        # Convert to ROS Image message (manually, without cv_bridge)
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
            self.get_logger().error(f'Error publishing heatmap: {e}')

    def _publish_markers(self):
        """Create and publish 3D pressure visualization markers for RViz."""
        marker_array = MarkerArray()

        # Delete old markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # Create markers for each grid cell with pressure
        marker_id = 0
        cell_width = self.mat_width / self.grid_size_x
        cell_height = self.mat_height / self.grid_size_y

        max_pressure = np.max(self.pressure_grid) if np.max(self.pressure_grid) > 0 else 1.0

        for y in range(self.grid_size_y):
            for x in range(self.grid_size_x):
                pressure = self.pressure_grid[y, x]

                if pressure > 0.01:  # Only show cells with significant pressure
                    marker = Marker()
                    marker.header.frame_id = 'world'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = 'pressure_cells'
                    marker.id = marker_id
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD

                    # Position (convert grid to world coordinates)
                    world_x = (x + 0.5) * cell_width - self.mat_width / 2.0
                    world_y = (y + 0.5) * cell_height - self.mat_height / 2.0
                    world_z = 0.002 + pressure * 0.01  # Height based on pressure

                    marker.pose.position.x = world_x
                    marker.pose.position.y = world_y
                    marker.pose.position.z = world_z
                    marker.pose.orientation.w = 1.0

                    # Scale
                    marker.scale.x = cell_width * 0.9
                    marker.scale.y = cell_height * 0.9
                    marker.scale.z = pressure * 0.02

                    # Color based on pressure (blue to red)
                    normalized_pressure = pressure / max_pressure
                    marker.color = self._pressure_to_color(normalized_pressure)

                    marker_array.markers.append(marker)
                    marker_id += 1

        self.marker_pub.publish(marker_array)

    def _pressure_to_color(self, normalized_pressure: float) -> ColorRGBA:
        """Convert normalized pressure (0-1) to color (blue=low, red=high)."""
        color = ColorRGBA()
        color.a = 0.8  # Transparency

        if normalized_pressure < 0.5:
            # Blue to green
            color.r = 0.0
            color.g = normalized_pressure * 2.0
            color.b = 1.0 - normalized_pressure * 2.0
        else:
            # Green to red
            color.r = (normalized_pressure - 0.5) * 2.0
            color.g = 1.0 - (normalized_pressure - 0.5) * 2.0
            color.b = 0.0

        return color


def main(args=None):
    rclpy.init(args=args)
    node = PressureMatVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
