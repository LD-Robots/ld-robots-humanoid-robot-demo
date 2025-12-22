"""State estimation helpers for WBC controller."""

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except Exception:
    PINOCCHIO_AVAILABLE = False


@dataclass
class RobotState:
    joint_positions: Dict[str, float]
    joint_velocities: Dict[str, float]
    com: Optional[np.ndarray]
    com_vel: Optional[np.ndarray]


class StateEstimator:
    def __init__(self, node, *, urdf_path: str, com_in_world: bool) -> None:
        self._node = node
        self._urdf_path = urdf_path
        self._com_in_world = com_in_world
        self._pin_model = None
        self._pin_data = None
        self._pin_joint_map: Dict[str, int] = {}
        self._has_floating_base = False

        if PINOCCHIO_AVAILABLE and urdf_path:
            try:
                if urdf_path.endswith('.xml'):
                    self._pin_model = pin.buildModelFromMJCF(urdf_path)
                else:
                    self._pin_model = pin.buildModelFromUrdf(urdf_path)
                self._pin_data = self._pin_model.createData()
                if len(self._pin_model.joints) > 1:
                    self._has_floating_base = self._pin_model.joints[1].nq == 7
                self._node.get_logger().info(f'Floating base: {self._has_floating_base}')
                for name in self._pin_model.names:
                    if name == 'universe':
                        continue
                    joint_id = self._pin_model.getJointId(name)
                    if joint_id >= 0:
                        idx = self._pin_model.joints[joint_id].idx_q
                        self._pin_joint_map[name] = idx
                self._node.get_logger().info(f'Pinocchio model loaded: {self._pin_model.nq} q')
                self._node.get_logger().info(
                    f'CoM frame: {"world" if self._com_in_world else "model"}'
                )
            except Exception as exc:
                self._node.get_logger().warn(f'Failed to load Pinocchio model: {exc}')

    def build_state(
        self,
        joint_state: Optional[JointState],
        base_pose_msg: Optional[PoseStamped],
    ) -> Optional[RobotState]:
        if joint_state is None:
            return None

        positions: Dict[str, float] = {}
        velocities: Dict[str, float] = {}
        for idx, name in enumerate(joint_state.name):
            positions[name] = joint_state.position[idx]
            if idx < len(joint_state.velocity):
                velocities[name] = joint_state.velocity[idx]

        com = None
        com_vel = None
        if self._pin_model is not None and self._pin_data is not None:
            q = np.zeros(self._pin_model.nq)
            dq = np.zeros(self._pin_model.nv)
            for joint_name, q_idx in self._pin_joint_map.items():
                if joint_name == 'floating_base_joint' and self._com_in_world:
                    continue
                if joint_name in positions:
                    q[q_idx] = positions[joint_name]
                if joint_name in velocities:
                    dq[q_idx] = velocities[joint_name]
            if self._has_floating_base and self._com_in_world and base_pose_msg is not None:
                pose = base_pose_msg.pose
                q[0:3] = [pose.position.x, pose.position.y, pose.position.z]
                quat = np.array(
                    [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                    dtype=float
                )
                norm = np.linalg.norm(quat)
                if norm > 1e-9:
                    quat = quat / norm
                q[3:7] = quat
            try:
                pin.forwardKinematics(self._pin_model, self._pin_data, q, dq)
                pin.centerOfMass(self._pin_model, self._pin_data, q, dq)
                com = self._pin_data.com[0].copy()
                com_vel = self._pin_data.vcom[0].copy()
            except Exception as exc:
                self._node.get_logger().warn(
                    f'Pinocchio computation failed: {exc}', throttle_duration_sec=5.0
                )

        return RobotState(positions, velocities, com, com_vel)
