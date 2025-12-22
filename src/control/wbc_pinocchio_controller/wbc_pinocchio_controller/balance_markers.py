"""Publish CoM/ZMP points and visualization markers."""

from typing import Optional

import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class BalanceMarkersPublisher:
    def __init__(
        self,
        node,
        *,
        com_in_world: bool,
        com_topic: str = 'com_position',
        zmp_topic: str = 'zmp_position',
        marker_topic: str = 'balance_markers',
    ) -> None:
        self._node = node
        self._com_in_world = com_in_world
        self._com_pub = node.create_publisher(PointStamped, com_topic, 10)
        self._zmp_pub = node.create_publisher(PointStamped, zmp_topic, 10)
        self._marker_pub = node.create_publisher(MarkerArray, marker_topic, 10)

    def publish(
        self,
        com_point_model: Optional[np.ndarray],
        base_pose_msg: Optional[PoseStamped],
    ) -> None:
        if com_point_model is None:
            return

        now = self._node.get_clock().now().to_msg()
        com_point = np.array(com_point_model, dtype=float)
        if not self._com_in_world and base_pose_msg is not None:
            pose = base_pose_msg.pose
            rot = self._quat_to_rot(pose.orientation)
            com_point = rot @ com_point + np.array(
                [pose.position.x, pose.position.y, pose.position.z], dtype=float
            )

        com_msg = PointStamped()
        com_msg.header.stamp = now
        com_msg.header.frame_id = 'world'
        com_msg.point.x = float(com_point[0])
        com_msg.point.y = float(com_point[1])
        com_msg.point.z = float(com_point[2])
        self._com_pub.publish(com_msg)

        zmp_msg = PointStamped()
        zmp_msg.header.stamp = now
        zmp_msg.header.frame_id = 'world'
        zmp_msg.point.x = float(com_point[0])
        zmp_msg.point.y = float(com_point[1])
        zmp_msg.point.z = 0.0
        self._zmp_pub.publish(zmp_msg)

        markers = MarkerArray()
        com_marker = Marker()
        com_marker.header = com_msg.header
        com_marker.ns = 'com'
        com_marker.id = 0
        com_marker.type = Marker.SPHERE
        com_marker.action = Marker.ADD
        com_marker.pose.position.x = com_msg.point.x
        com_marker.pose.position.y = com_msg.point.y
        com_marker.pose.position.z = com_msg.point.z
        com_marker.pose.orientation.w = 1.0
        com_marker.scale.x = 0.04
        com_marker.scale.y = 0.04
        com_marker.scale.z = 0.04
        com_marker.color.r = 0.1
        com_marker.color.g = 0.9
        com_marker.color.b = 0.1
        com_marker.color.a = 0.9

        zmp_marker = Marker()
        zmp_marker.header = zmp_msg.header
        zmp_marker.ns = 'zmp'
        zmp_marker.id = 1
        zmp_marker.type = Marker.SPHERE
        zmp_marker.action = Marker.ADD
        zmp_marker.pose.position.x = zmp_msg.point.x
        zmp_marker.pose.position.y = zmp_msg.point.y
        zmp_marker.pose.position.z = zmp_msg.point.z
        zmp_marker.pose.orientation.w = 1.0
        zmp_marker.scale.x = 0.05
        zmp_marker.scale.y = 0.05
        zmp_marker.scale.z = 0.05
        zmp_marker.color.r = 0.9
        zmp_marker.color.g = 0.1
        zmp_marker.color.b = 0.1
        zmp_marker.color.a = 0.9

        markers.markers.append(com_marker)
        markers.markers.append(zmp_marker)
        self._marker_pub.publish(markers)

    @staticmethod
    def _quat_to_rot(quat) -> np.ndarray:
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ])
