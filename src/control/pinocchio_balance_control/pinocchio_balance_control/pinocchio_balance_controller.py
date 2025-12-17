#!/usr/bin/env python3
"""
Pinocchio-based Balance Controller.
Uses Pinocchio rigid body dynamics for accurate CoM and ZMP calculations.
"""

import numpy as np
import pinocchio as pin
from pinocchio.utils import zero

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PointStamped, Point
import geometry_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory
import os


class PinocchioBalanceController(Node):
    """Advanced balance controller using Pinocchio dynamics."""

    def __init__(self):
        super().__init__('pinocchio_balance_controller')

        # Parameters
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('balance_kp', 3.0)
        self.declare_parameter('balance_kd', 2.5)
        self.declare_parameter('ankle_limit', 0.12)
        self.declare_parameter('hip_limit', 0.15)
        self.declare_parameter('knee_bend', 0.08)       # Reduced for more natural stance
        self.declare_parameter('stance_width', 0.06)    # Slightly narrower for stability

        # Advanced parameters
        self.declare_parameter('com_deadzone', 0.005)   # m - ignore errors < 5mm
        self.declare_parameter('filter_alpha', 0.4)     # Command smoothing (0=smooth, 1=responsive)
        self.declare_parameter('prediction_time', 0.2)  # s - look ahead time
        self.declare_parameter('auto_tune', False)      # Enable automatic gain tuning
        self.declare_parameter('auto_step', False)      # Automatically start a few steps
        self.declare_parameter('step_count', 4)         # How many steps to take
        self.declare_parameter('step_duration', 0.8)    # Duration of single swing (s)
        self.declare_parameter('shift_duration', 0.35)  # Duration of weight shift (s)
        self.declare_parameter('step_pitch_amp', 0.24)  # Hip pitch amplitude (rad)
        self.declare_parameter('step_knee_lift', 0.28)  # Knee lift amplitude (rad)
        self.declare_parameter('hip_roll_shift', 0.07)  # Lateral weight shift (rad)
        self.declare_parameter('walk_start_delay', 1.5) # Delay after start before walking (s)
        self.declare_parameter('walk_abort_margin', 1.5) # Stop walking if CoM margin explodes

        self.control_rate = self.get_parameter('control_rate').value
        self.balance_kp = self.get_parameter('balance_kp').value
        self.balance_kd = self.get_parameter('balance_kd').value
        self.ankle_limit = self.get_parameter('ankle_limit').value
        self.hip_limit = self.get_parameter('hip_limit').value
        self.knee_bend = self.get_parameter('knee_bend').value
        self.stance_width = self.get_parameter('stance_width').value
        self.com_deadzone = self.get_parameter('com_deadzone').value
        self.filter_alpha = self.get_parameter('filter_alpha').value
        self.prediction_time = self.get_parameter('prediction_time').value
        self.auto_tune = self.get_parameter('auto_tune').value
        # Allow overrides coming in as strings from launch substitutions
        def _to_bool(val):
            return val if isinstance(val, bool) else str(val).lower() in ('1', 'true', 'yes')

        def _to_float(val):
            try:
                return float(val)
            except Exception:
                return 0.0

        self.auto_step = _to_bool(self.get_parameter('auto_step').value)
        self.step_count = max(0, int(_to_float(self.get_parameter('step_count').value)))
        self.step_duration = _to_float(self.get_parameter('step_duration').value)
        self.shift_duration = _to_float(self.get_parameter('shift_duration').value)
        self.step_pitch_amp = _to_float(self.get_parameter('step_pitch_amp').value)
        self.step_knee_lift = _to_float(self.get_parameter('step_knee_lift').value)
        self.hip_roll_shift = _to_float(self.get_parameter('hip_roll_shift').value)
        self.walk_start_delay = _to_float(self.get_parameter('walk_start_delay').value)
        self.walk_abort_margin = _to_float(self.get_parameter('walk_abort_margin').value)

        # Load URDF
        urdf_path = self.get_parameter('urdf_path').value
        if not urdf_path:
            # Default path
            pkg_share = get_package_share_directory('humanoid_description')
            urdf_path = os.path.join(pkg_share, 'urdf', 'robot.xml')

        self.get_logger().info(f'Loading URDF from: {urdf_path}')

        try:
            # Load model with Pinocchio
            self.model = pin.buildModelFromMJCF(urdf_path)
            self.data = self.model.createData()

            self.get_logger().info(f'Model loaded: {self.model.nq} DOFs, {self.model.nv} velocities')
            self.get_logger().info(f'Robot mass: {pin.computeTotalMass(self.model):.2f} kg')

        except Exception as e:
            self.get_logger().error(f'Failed to load URDF: {e}')
            raise

        # State
        self.joint_state = None
        self.imu_data = None
        self.control_count = 0

        # Initialize joint configuration vectors
        self.q = pin.neutral(self.model)  # Joint positions
        self.dq = zero(self.model.nv)     # Joint velocities

        # CoM tracking
        self.com_position = np.zeros(3)
        self.com_velocity = np.zeros(3)
        self.com_position_prev = np.zeros(3)  # For velocity estimation
        self.zmp_position = np.zeros(2)  # x, y

        # Foot parameters (from robot geometry)
        self.foot_length = 0.12  # m
        self.foot_width = 0.08   # m

        # Command filtering
        self.ankle_pitch_filtered = 0.0
        self.ankle_roll_filtered = 0.0
        self.hip_pitch_filtered = 0.0
        self.hip_roll_filtered = 0.0

        # Auto-tuning state
        self.com_error_history = []
        self.max_history_length = 100
        self.tune_interval = 500  # Cycles between tuning adjustments

        # Walking state
        self.walk_state = 'idle'
        self.support_leg = 'left'  # Which leg is supporting during swing
        self.steps_completed = 0
        self.state_start_time = None
        self.first_control_time = None
        self.walk_offsets = {
            'left': {'hip_pitch': 0.0, 'hip_roll': 0.0, 'knee': 0.0, 'ankle_pitch': 0.0, 'ankle_roll': 0.0},
            'right': {'hip_pitch': 0.0, 'hip_roll': 0.0, 'knee': 0.0, 'ankle_pitch': 0.0, 'ankle_roll': 0.0},
        }
        self.max_walk_offsets = {
            'hip_pitch': 0.35,
            'hip_roll': 0.25,
            'knee': 0.5,
            'ankle_pitch': self.ankle_limit,
            'ankle_roll': self.ankle_limit,
        }

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

        # Publishers
        self.target_pub = self.create_publisher(JointState, 'target_positions', 10)
        self.com_pub = self.create_publisher(PointStamped, 'com_position', 10)
        self.zmp_pub = self.create_publisher(PointStamped, 'zmp_position', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'balance_markers', 10)

        self.get_logger().info('Pinocchio Balance Controller initialized')
        self.get_logger().info(f'Gains: Kp={self.balance_kp}, Kd={self.balance_kd}')

    def joint_callback(self, msg):
        """Receive joint states and update Pinocchio model."""
        self.joint_state = msg

        # Debug: Print joint names on first callback
        if not hasattr(self, 'joint_names_printed'):
            self.joint_names_printed = True
            self.get_logger().info(f'Received {len(msg.name)} joints: {msg.name[:5]}...')
            self.get_logger().info(f'Pinocchio model: nq={self.model.nq}, nv={self.model.nv}')

            # Create joint name to Pinocchio index mapping
            self.joint_map = {}
            for i, name in enumerate(msg.name):
                # Try to find this joint in Pinocchio model
                if self.model.existJointName(name):
                    joint_id = self.model.getJointId(name)
                    joint_idx = self.model.idx_qs[joint_id]
                    self.joint_map[i] = joint_idx
                    self.get_logger().info(f'Mapped {name} (ROS idx {i}) -> Pinocchio idx {joint_idx}')

        # Update joint positions using the mapping
        for ros_idx, pin_idx in self.joint_map.items():
            if ros_idx < len(msg.position):
                self.q[pin_idx] = msg.position[ros_idx]

        # Update joint velocities if available
        if len(msg.velocity) > 0:
            for ros_idx, pin_idx in self.joint_map.items():
                if ros_idx < len(msg.velocity):
                    self.dq[pin_idx] = msg.velocity[ros_idx]

        # Start control loop on first joint data
        if not hasattr(self, 'timer_started'):
            self.timer_started = True
            self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
            self.get_logger().info('Starting control loop')

    def imu_callback(self, msg):
        """Receive IMU data."""
        self.imu_data = msg

    def compute_dynamics(self):
        """Compute CoM position, velocity, and ZMP using Pinocchio."""
        # Forward kinematics
        pin.forwardKinematics(self.model, self.data, self.q, self.dq)

        # Compute CoM
        pin.centerOfMass(self.model, self.data, self.q, self.dq)
        self.com_position = self.data.com[0]  # CoM position
        self.com_velocity = self.data.vcom[0]  # CoM velocity

        # Compute ZMP (simplified - assumes feet on ground)
        # ZMP_x = CoM_x - (CoM_z / g) * CoM_ddot_x
        # For now, use simple approximation: ZMP ≈ horizontal CoM projection
        self.zmp_position[0] = self.com_position[0]
        self.zmp_position[1] = self.com_position[1]

    def _time_now(self):
        """Return current time in seconds (sim time aware)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _reset_walk_offsets(self):
        self.walk_offsets = {
            'left': {'hip_pitch': 0.0, 'hip_roll': 0.0, 'knee': 0.0, 'ankle_pitch': 0.0, 'ankle_roll': 0.0},
            'right': {'hip_pitch': 0.0, 'hip_roll': 0.0, 'knee': 0.0, 'ankle_pitch': 0.0, 'ankle_roll': 0.0},
        }

    def _clamp_offset(self, val, key):
        limit = self.max_walk_offsets.get(key, 1.0)
        return float(np.clip(val, -limit, limit))

    def _set_walk_state(self, state, support_leg, now):
        self.walk_state = state
        self.support_leg = support_leg
        self.state_start_time = now
        self.get_logger().info(
            f'Walking state -> {state} (support: {support_leg}, step {self.steps_completed}/{self.step_count})'
        )

    def _update_walking(self, now):
        """Simple finite state machine to generate swing offsets."""
        self._reset_walk_offsets()

        if not self.auto_step or self.steps_completed >= self.step_count:
            return

        if self.first_control_time is None:
            self.first_control_time = now

        # Wait before starting to give balance a moment to settle
        if now - self.first_control_time < self.walk_start_delay:
            return

        # Initialize walking
        if self.walk_state == 'idle':
            self._set_walk_state('shift_left', 'left', now)
            return

        state_elapsed = now - (self.state_start_time or now)

        if self.walk_state.startswith('shift'):
            # Weight shift toward support leg
            direction = 1.0 if self.support_leg == 'left' else -1.0
            progress = min(1.0, state_elapsed / max(self.shift_duration, 1e-3))
            shift = direction * self.hip_roll_shift * progress
            self.walk_offsets[self.support_leg]['hip_roll'] = shift
            self.walk_offsets['left' if self.support_leg == 'right' else 'right']['hip_roll'] = -shift

            if state_elapsed >= self.shift_duration:
                # Move into swing of the opposite leg
                swing_leg = 'right' if self.support_leg == 'left' else 'left'
                self._set_walk_state(f'swing_{swing_leg}', self.support_leg, now)
            return

        if self.walk_state.startswith('swing'):
            swing_leg = 'left' if 'left' in self.walk_state else 'right'
            support_leg = 'right' if swing_leg == 'left' else 'left'
            progress = min(1.0, state_elapsed / max(self.step_duration, 1e-3))

            # Forward swing using smooth profile: -amp -> +amp
            hip_pitch = self._clamp_offset(self.step_pitch_amp * (2.0 * progress - 1.0), 'hip_pitch')
            knee = self._clamp_offset(self.step_knee_lift * np.sin(np.pi * progress), 'knee')
            ankle_pitch = self._clamp_offset(-0.5 * knee, 'ankle_pitch')  # keep foot roughly level

            # Keep weight over support leg
            shift = self.hip_roll_shift
            self.walk_offsets[support_leg]['hip_roll'] = self._clamp_offset(
                shift if support_leg == 'left' else -shift, 'hip_roll'
            )
            self.walk_offsets[swing_leg]['hip_roll'] = self._clamp_offset(
                -shift if support_leg == 'left' else shift, 'hip_roll'
            )

            # Apply swing offsets to swing leg
            self.walk_offsets[swing_leg]['hip_pitch'] = hip_pitch
            self.walk_offsets[swing_leg]['knee'] = knee
            self.walk_offsets[swing_leg]['ankle_pitch'] = ankle_pitch

            if state_elapsed >= self.step_duration:
                self.steps_completed += 1
                self.get_logger().info(f'Completed step {self.steps_completed}/{self.step_count}')

                if self.steps_completed >= self.step_count:
                    self._set_walk_state('idle', support_leg, now)
                else:
                    # Next shift onto the opposite leg to prepare next swing
                    next_support = swing_leg
                    self._set_walk_state(f'shift_{"left" if next_support == "left" else "right"}', next_support, now)
    def publish_visualization_markers(self):
        """Publish visualization markers for CoM, ZMP, and support polygon."""
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # CoM Marker (Red Sphere)
        com_marker = Marker()
        com_marker.header.frame_id = 'world'
        com_marker.header.stamp = stamp
        com_marker.ns = 'balance'
        com_marker.id = 0
        com_marker.type = Marker.SPHERE
        com_marker.action = Marker.ADD
        com_marker.pose.position.x = float(self.com_position[0])
        com_marker.pose.position.y = float(self.com_position[1])
        com_marker.pose.position.z = float(self.com_position[2])
        com_marker.pose.orientation.w = 1.0
        com_marker.scale.x = 0.10  # Larger for visibility
        com_marker.scale.y = 0.10
        com_marker.scale.z = 0.10
        com_marker.color.r = 1.0
        com_marker.color.g = 0.0
        com_marker.color.b = 0.0
        com_marker.color.a = 1.0
        marker_array.markers.append(com_marker)

        # ZMP Marker (Green Sphere)
        zmp_marker = Marker()
        zmp_marker.header.frame_id = 'world'
        zmp_marker.header.stamp = stamp
        zmp_marker.ns = 'balance'
        zmp_marker.id = 1
        zmp_marker.type = Marker.SPHERE
        zmp_marker.action = Marker.ADD
        zmp_marker.pose.position.x = float(self.zmp_position[0])
        zmp_marker.pose.position.y = float(self.zmp_position[1])
        zmp_marker.pose.position.z = 0.01
        zmp_marker.pose.orientation.w = 1.0
        zmp_marker.scale.x = 0.08  # Larger for visibility
        zmp_marker.scale.y = 0.08
        zmp_marker.scale.z = 0.02
        zmp_marker.color.r = 0.0
        zmp_marker.color.g = 1.0
        zmp_marker.color.b = 0.0
        zmp_marker.color.a = 1.0
        marker_array.markers.append(zmp_marker)

        # Support Polygon (Blue Rectangle)
        support_marker = Marker()
        support_marker.header.frame_id = 'world'
        support_marker.header.stamp = stamp
        support_marker.ns = 'balance'
        support_marker.id = 2
        support_marker.type = Marker.LINE_STRIP
        support_marker.action = Marker.ADD
        support_marker.pose.orientation.w = 1.0
        support_marker.scale.x = 0.02  # Line width (thicker for visibility)

        # Define support polygon corners
        half_length = self.foot_length / 2
        half_width = self.foot_width / 2
        corners = [
            (-half_length, -half_width, 0.0),
            (half_length, -half_width, 0.0),
            (half_length, half_width, 0.0),
            (-half_length, half_width, 0.0),
            (-half_length, -half_width, 0.0),  # Close the loop
        ]

        for x, y, z in corners:
            point = geometry_msgs.msg.Point()
            point.x = float(x)
            point.y = float(y)
            point.z = float(z)
            support_marker.points.append(point)

        support_marker.color.r = 0.0
        support_marker.color.g = 0.0
        support_marker.color.b = 1.0
        support_marker.color.a = 0.5
        marker_array.markers.append(support_marker)

        # CoM velocity vector (Yellow Arrow)
        vel_magnitude = np.sqrt(self.com_velocity[0]**2 + self.com_velocity[1]**2)
        if vel_magnitude > 0.01:  # Only show if moving
            vel_marker = Marker()
            vel_marker.header.frame_id = 'world'
            vel_marker.header.stamp = stamp
            vel_marker.ns = 'balance'
            vel_marker.id = 3
            vel_marker.type = Marker.ARROW
            vel_marker.action = Marker.ADD

            # Arrow from CoM in velocity direction
            start = geometry_msgs.msg.Point()
            start.x = float(self.com_position[0])
            start.y = float(self.com_position[1])
            start.z = float(self.com_position[2])

            end = geometry_msgs.msg.Point()
            scale = 0.2  # Scale velocity for visualization
            end.x = float(self.com_position[0] + self.com_velocity[0] * scale)
            end.y = float(self.com_position[1] + self.com_velocity[1] * scale)
            end.z = float(self.com_position[2])

            vel_marker.points = [start, end]
            vel_marker.scale.x = 0.01  # Shaft diameter
            vel_marker.scale.y = 0.02  # Head diameter
            vel_marker.color.r = 1.0
            vel_marker.color.g = 1.0
            vel_marker.color.b = 0.0
            vel_marker.color.a = 1.0
            marker_array.markers.append(vel_marker)

        self.marker_pub.publish(marker_array)

    def control_loop(self):
        """Main control loop - compute balance corrections."""
        if self.joint_state is None:
            if self.control_count == 0:
                self.get_logger().warn('Waiting for joint_states...')
            return

        if self.imu_data is None:
            if self.control_count == 0:
                self.get_logger().warn('Waiting for IMU data...')
            return

        self.control_count += 1
        now = self._time_now()

        # Debug: confirm we're running
        if self.control_count == 1:
            self.get_logger().info('Control loop running! Starting visualization...')

        # Compute dynamics
        self.compute_dynamics()

        # Update simple walking pattern (adds offsets to swing leg)
        self._update_walking(now)

        # Publish visualization markers
        if self.control_count % 5 == 0:  # Publish at 20 Hz
            try:
                self.publish_visualization_markers()
                if self.control_count == 5:
                    self.get_logger().info('First markers published successfully!')
            except Exception as e:
                self.get_logger().error(f'Failed to publish markers: {e}')

            # Also publish PointStamped for compatibility
            com_msg = PointStamped()
            com_msg.header.stamp = self.get_clock().now().to_msg()
            com_msg.header.frame_id = 'world'
            com_msg.point.x = float(self.com_position[0])
            com_msg.point.y = float(self.com_position[1])
            com_msg.point.z = float(self.com_position[2])
            self.com_pub.publish(com_msg)

            zmp_msg = PointStamped()
            zmp_msg.header.stamp = self.get_clock().now().to_msg()
            zmp_msg.header.frame_id = 'world'
            zmp_msg.point.x = float(self.zmp_position[0])
            zmp_msg.point.y = float(self.zmp_position[1])
            zmp_msg.point.z = 0.0
            self.zmp_pub.publish(zmp_msg)

        # Calculate support polygon center (assuming symmetric stance)
        support_center_x = 0.0  # Centered between feet
        support_center_y = 0.0

        # Calculate CoM error relative to support polygon
        com_error_x = self.com_position[0] - support_center_x
        com_error_y = self.com_position[1] - support_center_y

        # Apply deadzone to reduce micro-oscillations
        if abs(com_error_x) < self.com_deadzone:
            com_error_x = 0.0
        if abs(com_error_y) < self.com_deadzone:
            com_error_y = 0.0

        # Calculate support polygon limits
        support_limit_x = self.foot_length / 2  # ±6cm
        support_limit_y = self.foot_width / 2   # ±4cm

        # Calculate margins (0=centered, 1=at edge, >1=outside)
        com_margin_x = abs(com_error_x) / support_limit_x
        com_margin_y = abs(com_error_y) / support_limit_y

        # Predict future CoM position for anticipatory control
        com_predicted_x = self.com_position[0] + self.com_velocity[0] * self.prediction_time
        com_predicted_y = self.com_position[1] + self.com_velocity[1] * self.prediction_time

        # Use predicted error for more proactive control
        com_error_predicted_x = com_predicted_x - support_center_x
        com_error_predicted_y = com_predicted_y - support_center_y

        # Blend current and predicted error (70% current, 30% predicted)
        com_error_blend_x = 0.7 * com_error_x + 0.3 * com_error_predicted_x
        com_error_blend_y = 0.7 * com_error_y + 0.3 * com_error_predicted_y

        # Abort walking if CoM is far outside support polygon
        if self.walk_state != 'idle':
            if com_margin_x > self.walk_abort_margin or com_margin_y > self.walk_abort_margin:
                self.get_logger().warn(
                    f'CoM margin too high (x={com_margin_x:.2f}, y={com_margin_y:.2f}) - aborting walk'
                )
                self._set_walk_state('idle', self.support_leg, self._time_now())
                self.steps_completed = self.step_count
                self._reset_walk_offsets()

        # Auto-tuning: track error history and adjust gains
        if self.auto_tune:
            self.com_error_history.append(np.sqrt(com_error_x**2 + com_error_y**2))
            if len(self.com_error_history) > self.max_history_length:
                self.com_error_history.pop(0)

            if self.control_count % self.tune_interval == 0 and len(self.com_error_history) > 50:
                # Calculate error statistics
                error_mean = np.mean(self.com_error_history)
                error_std = np.std(self.com_error_history)

                # If error is consistently high, increase Kp
                if error_mean > 0.02:  # > 2cm average error
                    self.balance_kp = min(self.balance_kp * 1.1, 10.0)
                    self.get_logger().info(f'Auto-tune: Increased Kp to {self.balance_kp:.2f}')

                # If oscillating (high std), increase Kd
                if error_std > 0.01:  # High variance
                    self.balance_kd = min(self.balance_kd * 1.1, 5.0)
                    self.get_logger().info(f'Auto-tune: Increased Kd to {self.balance_kd:.2f}')

        # PD control on CoM position and velocity
        correction_x = -(self.balance_kp * com_error_blend_x + self.balance_kd * self.com_velocity[0])
        correction_y = -(self.balance_kp * com_error_blend_y + self.balance_kd * self.com_velocity[1])

        # Map corrections to ankle and hip commands
        # Ankle strategy for small tilts, hip strategy for larger tilts
        com_tilt_mag = np.sqrt(com_margin_x**2 + com_margin_y**2)
        hip_ratio = np.tanh(com_tilt_mag / 0.3)  # Transition around 30% of support
        ankle_ratio = 1.0 - 0.5 * hip_ratio

        # Joint sign conventions (inverted for this robot)
        ankle_pitch_raw = np.clip(-ankle_ratio * correction_x, -self.ankle_limit, self.ankle_limit)
        ankle_roll_raw = np.clip(-ankle_ratio * correction_y, -self.ankle_limit, self.ankle_limit)
        hip_pitch_raw = np.clip(-hip_ratio * correction_x, -self.hip_limit, self.hip_limit)
        hip_roll_raw = np.clip(-hip_ratio * correction_y, -self.hip_limit, self.hip_limit)

        # Apply exponential smoothing for fine, smooth movements
        self.ankle_pitch_filtered = self.filter_alpha * ankle_pitch_raw + (1 - self.filter_alpha) * self.ankle_pitch_filtered
        self.ankle_roll_filtered = self.filter_alpha * ankle_roll_raw + (1 - self.filter_alpha) * self.ankle_roll_filtered
        self.hip_pitch_filtered = self.filter_alpha * hip_pitch_raw + (1 - self.filter_alpha) * self.hip_pitch_filtered
        self.hip_roll_filtered = self.filter_alpha * hip_roll_raw + (1 - self.filter_alpha) * self.hip_roll_filtered

        # Use filtered values
        ankle_pitch = self.ankle_pitch_filtered
        ankle_roll = self.ankle_roll_filtered
        hip_pitch = self.hip_pitch_filtered
        hip_roll = self.hip_roll_filtered

        w_left = self.walk_offsets['left']
        w_right = self.walk_offsets['right']

        # Build command message
        target_msg = JointState()
        target_msg.header.stamp = self.get_clock().now().to_msg()

        target_msg.name = [
            'left_hip_pitch_joint',
            'left_hip_roll_joint',
            'left_knee_joint',
            'left_ankle_pitch_joint',
            'left_ankle_roll_joint',
            'right_hip_pitch_joint',
            'right_hip_roll_joint',
            'right_knee_joint',
            'right_ankle_pitch_joint',
            'right_ankle_roll_joint',
        ]

        target_msg.position = [
            hip_pitch + w_left['hip_pitch'],                               # left_hip_pitch
            self.stance_width + hip_roll + w_left['hip_roll'],             # left_hip_roll (outward)
            self.knee_bend + w_left['knee'],                               # left_knee
            -self.knee_bend/2 + ankle_pitch + w_left['ankle_pitch'],       # left_ankle_pitch
            ankle_roll + w_left['ankle_roll'],                             # left_ankle_roll
            hip_pitch + w_right['hip_pitch'],                              # right_hip_pitch
            -self.stance_width - hip_roll + w_right['hip_roll'],           # right_hip_roll (outward, opposite)
            self.knee_bend + w_right['knee'],                              # right_knee
            -self.knee_bend/2 + ankle_pitch + w_right['ankle_pitch'],      # right_ankle_pitch
            -ankle_roll + w_right['ankle_roll'],                           # right_ankle_roll (opposite)
        ]

        self.target_pub.publish(target_msg)

        # Debug output
        if self.control_count <= 5 or self.control_count % 50 == 0:
            self.get_logger().info(
                f'[{self.control_count}] CoM: [{self.com_position[0]:.3f}, {self.com_position[1]:.3f}, {self.com_position[2]:.3f}] | '
                f'Margin: {com_margin_x:.2f}/{com_margin_y:.2f} | '
                f'Correction: x={correction_x:.3f}, y={correction_y:.3f} | '
                f'Commands: ankle_pitch={ankle_pitch:.3f}, hip_pitch={hip_pitch:.3f}'
            )


def main(args=None):
    rclpy.init(args=args)
    controller = PinocchioBalanceController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
