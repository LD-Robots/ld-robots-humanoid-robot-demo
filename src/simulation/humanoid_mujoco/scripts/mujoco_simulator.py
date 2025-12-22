#!/usr/bin/env python3
"""
MuJoCo Simulator for Humanoid Robot with ROS 2 Integration.
This node runs the MuJoCo physics simulation and publishes robot state to ROS 2.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import WrenchStamped, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster
import time

try:
    import mujoco
    import mujoco.viewer
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False
    print("WARNING: MuJoCo not installed. Install with: pip install mujoco")


class MuJoCoSimulator(Node):
    """MuJoCo physics simulator with ROS 2 integration."""

    def __init__(self):
        super().__init__('mujoco_simulator')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('use_viewer', True)
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('realtime_factor', 1.0)
        self.declare_parameter('torso_body_name', '')
        self.declare_parameter('pelvis_body_name', '')

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.use_viewer = self.get_parameter('use_viewer').get_parameter_value().bool_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.realtime_factor = self.get_parameter('realtime_factor').get_parameter_value().double_value
        self.torso_body_name = self.get_parameter('torso_body_name').get_parameter_value().string_value
        self.pelvis_body_name = self.get_parameter('pelvis_body_name').get_parameter_value().string_value

        if not MUJOCO_AVAILABLE:
            self.get_logger().error('MuJoCo is not installed!')
            return

        # Load MuJoCo model
        try:
            self.get_logger().info(f'Loading MuJoCo model: {self.model_path}')
            self.model = mujoco.MjModel.from_xml_path(self.model_path)
            self.data = mujoco.MjData(self.model)
            self.get_logger().info(f'Model loaded successfully')
            self.get_logger().info(f'Number of joints: {self.model.njnt}')
            self.get_logger().info(f'Number of actuators: {self.model.nu}')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            return

        self.torso_body_id, self.torso_body_name = self._resolve_body(
            self.torso_body_name,
            ['torso', 'torso_link', 'pelvis', 'base_link'],
            'torso'
        )
        self.pelvis_body_id, self.pelvis_body_name = self._resolve_body(
            self.pelvis_body_name,
            ['pelvis', 'base_link', 'torso', 'torso_link'],
            'pelvis'
        )

        # ROS 2 Publishers
        self.clock_pub = self.create_publisher(Clock, 'clock', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.left_foot_force_pub = self.create_publisher(WrenchStamped, 'left_foot/wrench', 10)
        self.right_foot_force_pub = self.create_publisher(WrenchStamped, 'right_foot/wrench', 10)
        self.torso_pose_pub = self.create_publisher(PoseStamped, 'torso/pose', 10)
        self.pelvis_pose_pub = self.create_publisher(PoseStamped, 'pelvis/pose', 10)

        # ROS 2 Subscribers
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_command_callback,
            10
        )

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Control
        self.joint_commands = np.zeros(self.model.nu)

        # Timing
        self.dt = self.model.opt.timestep
        self.publish_period = 1.0 / self.publish_rate

        # Publish every N simulation steps (deterministic)
        physics_rate = 1.0 / self.dt  # Hz
        self.publish_every_n_steps = max(1, int(physics_rate / self.publish_rate))
        self.step_counter = 0

        # Simulation thread
        self.simulation_active = True

        self.get_logger().info(f'Physics: {physics_rate} Hz, Publishing every {self.publish_every_n_steps} steps')

        self.get_logger().info('MuJoCo Simulator initialized')

    def _resolve_body(self, preferred_name, fallback_names, label):
        candidates = []
        if preferred_name:
            candidates.append(preferred_name)
        candidates.extend(fallback_names)

        for name in candidates:
            torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if torso_id >= 0:
                self.get_logger().info(f'Using {label} body for TF/pose: {name}')
                return torso_id, name

        self.get_logger().warn(f'No {label} body found for pose/TF publishing')
        return None, ''

    def joint_command_callback(self, msg):
        """Receive joint commands from ROS 2."""
        if len(msg.data) == self.model.nu:
            self.joint_commands = np.array(msg.data)
        else:
            self.get_logger().warn(f'Expected {self.model.nu} commands, got {len(msg.data)}')

    def step_simulation(self):
        """Step the MuJoCo simulation."""
        # Apply control
        self.data.ctrl[:] = self.joint_commands

        # Step simulation
        mujoco.mj_step(self.model, self.data)

    def publish_state(self):
        """Publish simulation state to ROS 2."""
        if not MUJOCO_AVAILABLE:
            return

        # Publish simulation clock
        clock_msg = Clock()
        sim_time_sec = int(self.data.time)
        sim_time_nsec = int((self.data.time - sim_time_sec) * 1e9)
        clock_msg.clock.sec = sim_time_sec
        clock_msg.clock.nanosec = sim_time_nsec
        self.clock_pub.publish(clock_msg)

        # Use simulation time for all timestamps
        current_time = self.get_clock().now()

        # Publish joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = current_time.to_msg()
        joint_state_msg.header.frame_id = 'world'

        # Get joint names and states
        for i in range(self.model.njnt):
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if joint_name:
                joint_state_msg.name.append(joint_name)

                # Position
                qpos_addr = self.model.jnt_qposadr[i]
                joint_state_msg.position.append(float(self.data.qpos[qpos_addr]))

                # Velocity
                qvel_addr = self.model.jnt_dofadr[i]
                joint_state_msg.velocity.append(float(self.data.qvel[qvel_addr]))

                # Effort (from actuator force)
                joint_state_msg.effort.append(0.0)  # TODO: Get from actuator force

        self.joint_state_pub.publish(joint_state_msg)

        # Publish IMU data
        self.publish_imu(current_time)

        # Publish foot forces
        self.publish_foot_forces(current_time)

        # Publish torso/pelvis pose
        self.publish_torso_pose(current_time)
        self.publish_pelvis_pose(current_time)

        # Publish TF
        self.publish_tf(current_time)

    def publish_imu(self, current_time):
        """Publish IMU sensor data."""
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Find IMU sensors (using torso sensors as primary)
        accel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'imu-torso-linear-acceleration')
        gyro_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'imu-torso-angular-velocity')
        imu_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'imu_in_torso')

        if accel_id >= 0:
            accel_addr = self.model.sensor_adr[accel_id]
            imu_msg.linear_acceleration.x = float(self.data.sensordata[accel_addr])
            imu_msg.linear_acceleration.y = float(self.data.sensordata[accel_addr + 1])
            imu_msg.linear_acceleration.z = float(self.data.sensordata[accel_addr + 2])

        if gyro_id >= 0:
            gyro_addr = self.model.sensor_adr[gyro_id]
            imu_msg.angular_velocity.x = float(self.data.sensordata[gyro_addr])
            imu_msg.angular_velocity.y = float(self.data.sensordata[gyro_addr + 1])
            imu_msg.angular_velocity.z = float(self.data.sensordata[gyro_addr + 2])

        # Provide orientation from the IMU site (closest to actual sensor frame)
        if imu_site_id >= 0:
            # Site rotation matrix (row-major 3x3)
            r = self.data.xmat[imu_site_id].reshape((3, 3))
            imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = self.rotmat_to_quat(r)
        elif self.torso_body_id is not None:
            # Fallback to torso orientation
            quat = self.data.xquat[self.torso_body_id]  # w, x, y, z
            imu_msg.orientation.w = float(quat[0])
            imu_msg.orientation.x = float(quat[1])
            imu_msg.orientation.y = float(quat[2])
            imu_msg.orientation.z = float(quat[3])

        self.imu_pub.publish(imu_msg)

    @staticmethod
    def rotmat_to_quat(r):
        """Convert 3x3 rotation matrix to quaternion (x, y, z, w)."""
        tr = r[0, 0] + r[1, 1] + r[2, 2]
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (r[2, 1] - r[1, 2]) / S
            qy = (r[0, 2] - r[2, 0]) / S
            qz = (r[1, 0] - r[0, 1]) / S
        elif (r[0, 0] > r[1, 1]) and (r[0, 0] > r[2, 2]):
            S = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
            qw = (r[2, 1] - r[1, 2]) / S
            qx = 0.25 * S
            qy = (r[0, 1] + r[1, 0]) / S
            qz = (r[0, 2] + r[2, 0]) / S
        elif r[1, 1] > r[2, 2]:
            S = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
            qw = (r[0, 2] - r[2, 0]) / S
            qx = (r[0, 1] + r[1, 0]) / S
            qy = 0.25 * S
            qz = (r[1, 2] + r[2, 1]) / S
        else:
            S = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
            qw = (r[1, 0] - r[0, 1]) / S
            qx = (r[0, 2] + r[2, 0]) / S
            qy = (r[1, 2] + r[2, 1]) / S
            qz = 0.25 * S
        return float(qx), float(qy), float(qz), float(qw)

    def publish_foot_forces(self, current_time):
        """Publish foot force/torque sensor data."""
        # Left foot
        left_force_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'left_foot_force')
        left_torque_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'left_foot_torque')

        if left_force_id >= 0:
            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = current_time.to_msg()
            wrench_msg.header.frame_id = 'left_foot'

            force_addr = self.model.sensor_adr[left_force_id]
            wrench_msg.wrench.force.x = float(self.data.sensordata[force_addr])
            wrench_msg.wrench.force.y = float(self.data.sensordata[force_addr + 1])
            wrench_msg.wrench.force.z = float(self.data.sensordata[force_addr + 2])

            if left_torque_id >= 0:
                torque_addr = self.model.sensor_adr[left_torque_id]
                wrench_msg.wrench.torque.x = float(self.data.sensordata[torque_addr])
                wrench_msg.wrench.torque.y = float(self.data.sensordata[torque_addr + 1])
                wrench_msg.wrench.torque.z = float(self.data.sensordata[torque_addr + 2])

            self.left_foot_force_pub.publish(wrench_msg)

        # Right foot
        right_force_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'right_foot_force')
        right_torque_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'right_foot_torque')

        if right_force_id >= 0:
            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = current_time.to_msg()
            wrench_msg.header.frame_id = 'right_foot'

            force_addr = self.model.sensor_adr[right_force_id]
            wrench_msg.wrench.force.x = float(self.data.sensordata[force_addr])
            wrench_msg.wrench.force.y = float(self.data.sensordata[force_addr + 1])
            wrench_msg.wrench.force.z = float(self.data.sensordata[force_addr + 2])

            if right_torque_id >= 0:
                torque_addr = self.model.sensor_adr[right_torque_id]
                wrench_msg.wrench.torque.x = float(self.data.sensordata[torque_addr])
                wrench_msg.wrench.torque.y = float(self.data.sensordata[torque_addr + 1])
                wrench_msg.wrench.torque.z = float(self.data.sensordata[torque_addr + 2])

            self.right_foot_force_pub.publish(wrench_msg)

    def publish_torso_pose(self, current_time):
        """Publish torso pose."""
        if self.torso_body_id is not None:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = current_time.to_msg()
            pose_msg.header.frame_id = 'world'

            # Use torso center of mass for a centered TF/pose.
            pose_msg.pose.position.x = float(self.data.xipos[self.torso_body_id][0])
            pose_msg.pose.position.y = float(self.data.xipos[self.torso_body_id][1])
            pose_msg.pose.position.z = float(self.data.xipos[self.torso_body_id][2])

            # Quaternion (w, x, y, z in MuJoCo)
            quat = self.data.xquat[self.torso_body_id]
            pose_msg.pose.orientation.w = float(quat[0])
            pose_msg.pose.orientation.x = float(quat[1])
            pose_msg.pose.orientation.y = float(quat[2])
            pose_msg.pose.orientation.z = float(quat[3])

            self.torso_pose_pub.publish(pose_msg)

    def publish_pelvis_pose(self, current_time):
        """Publish pelvis pose."""
        if self.pelvis_body_id is not None:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = current_time.to_msg()
            pose_msg.header.frame_id = 'world'

            pose_msg.pose.position.x = float(self.data.xpos[self.pelvis_body_id][0])
            pose_msg.pose.position.y = float(self.data.xpos[self.pelvis_body_id][1])
            pose_msg.pose.position.z = float(self.data.xpos[self.pelvis_body_id][2])

            quat = self.data.xquat[self.pelvis_body_id]
            pose_msg.pose.orientation.w = float(quat[0])
            pose_msg.pose.orientation.x = float(quat[1])
            pose_msg.pose.orientation.y = float(quat[2])
            pose_msg.pose.orientation.z = float(quat[3])

            self.pelvis_pose_pub.publish(pose_msg)

    def publish_tf(self, current_time):
        """Publish TF transforms."""
        if self.pelvis_body_id is not None:
            pelvis_tf = TransformStamped()
            pelvis_tf.header.stamp = current_time.to_msg()
            pelvis_tf.header.frame_id = 'world'
            pelvis_tf.child_frame_id = 'pelvis'
            pelvis_tf.transform.translation.x = float(self.data.xpos[self.pelvis_body_id][0])
            pelvis_tf.transform.translation.y = float(self.data.xpos[self.pelvis_body_id][1])
            pelvis_tf.transform.translation.z = float(self.data.xpos[self.pelvis_body_id][2])

            pelvis_quat = self.data.xquat[self.pelvis_body_id]
            pelvis_tf.transform.rotation.w = float(pelvis_quat[0])
            pelvis_tf.transform.rotation.x = float(pelvis_quat[1])
            pelvis_tf.transform.rotation.y = float(pelvis_quat[2])
            pelvis_tf.transform.rotation.z = float(pelvis_quat[3])

            self.tf_broadcaster.sendTransform(pelvis_tf)

        if self.torso_body_id is not None:
            torso_tf = TransformStamped()
            torso_tf.header.stamp = current_time.to_msg()
            torso_tf.header.frame_id = 'world'
            torso_tf.child_frame_id = 'torso'
            torso_tf.transform.translation.x = float(self.data.xipos[self.torso_body_id][0])
            torso_tf.transform.translation.y = float(self.data.xipos[self.torso_body_id][1])
            torso_tf.transform.translation.z = float(self.data.xipos[self.torso_body_id][2])

            torso_quat = self.data.xquat[self.torso_body_id]
            torso_tf.transform.rotation.w = float(torso_quat[0])
            torso_tf.transform.rotation.x = float(torso_quat[1])
            torso_tf.transform.rotation.y = float(torso_quat[2])
            torso_tf.transform.rotation.z = float(torso_quat[3])

            self.tf_broadcaster.sendTransform(torso_tf)

    def run_with_viewer(self):
        """Run simulation with MuJoCo viewer."""
        self.get_logger().info('Starting MuJoCo viewer...')

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running() and rclpy.ok():
                step_start = time.time()

                # Step simulation
                self.step_simulation()
                self.step_counter += 1

                # Publish state every N steps (deterministic)
                if self.step_counter >= self.publish_every_n_steps:
                    self.publish_state()
                    self.step_counter = 0

                # Sync viewer
                viewer.sync()

                # Maintain realtime
                time_until_next_step = self.dt * self.realtime_factor - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

                # Process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0)

    def run_headless(self):
        """Run simulation without viewer."""
        self.get_logger().info('Starting headless simulation...')

        while rclpy.ok():
            step_start = time.time()

            # Step simulation
            self.step_simulation()
            self.step_counter += 1

            # Publish state every N steps (deterministic)
            if self.step_counter >= self.publish_every_n_steps:
                self.publish_state()
                self.step_counter = 0

            # Maintain realtime
            time_until_next_step = self.dt * self.realtime_factor - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            # Process ROS callbacks
            rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
    rclpy.init(args=args)

    if not MUJOCO_AVAILABLE:
        print("ERROR: MuJoCo is not installed!")
        print("Install with: pip install mujoco")
        return

    simulator = MuJoCoSimulator()

    try:
        if simulator.use_viewer:
            simulator.run_with_viewer()
        else:
            simulator.run_headless()
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
