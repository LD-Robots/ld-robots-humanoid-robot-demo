#!/usr/bin/env python3
"""
Camera image processor node for the humanoid robot.
Processes camera images for object detection, face recognition, etc.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraProcessor(Node):
    """Process camera images for perception tasks."""

    def __init__(self):
        super().__init__('camera_processor')

        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_processed')
        self.declare_parameter('enable_visualization', True)

        camera_topic = self.get_parameter('camera_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        # Publishers
        self.image_pub = self.create_publisher(
            Image,
            output_topic,
            10
        )

        self.get_logger().info('Camera processor node started')

    def image_callback(self, msg: Image):
        """Process incoming camera images."""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process image (placeholder - add your vision algorithms here)
            processed_image = self.process_image(cv_image)

            # Convert back to ROS Image and publish
            output_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            output_msg.header = msg.header
            self.image_pub.publish(output_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def process_image(self, image):
        """
        Process the image with vision algorithms.

        Args:
            image: OpenCV image (numpy array)

        Returns:
            Processed image
        """
        # Placeholder processing - add your algorithms here
        # Examples:
        # - Object detection
        # - Face recognition
        # - Pose estimation
        # - Semantic segmentation

        # Simple edge detection as example
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Convert back to BGR for visualization
        processed = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        return processed


def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
