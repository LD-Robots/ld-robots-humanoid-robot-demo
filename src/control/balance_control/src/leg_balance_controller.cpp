#include "balance_control/leg_balance_controller.hpp"
#include <cmath>
#include <algorithm>

namespace balance_control
{

LegBalanceController::LegBalanceController(const rclcpp::NodeOptions & options)
: Node("leg_balance_controller", options),
  zmp_stable_(false),
  model_loaded_(false),
  joint_state_received_(false),
  is_getting_up_(false)
{
  // Declare and get parameters
  this->declare_parameter("left_leg_chain_base", "base");
  this->declare_parameter("left_leg_chain_tip", "LFootBushing_GPF_1517_12");
  this->declare_parameter("right_leg_chain_base", "base");
  this->declare_parameter("right_leg_chain_tip", "RFootBushing_GPF_1517_12");
  this->declare_parameter("control_rate", 100.0);
  this->declare_parameter("ankle_torque_to_position_gain", 0.005);
  this->declare_parameter("hip_torque_to_position_gain", 0.005);
  this->declare_parameter("ankle_torque_deadzone", 0.5);
  this->declare_parameter("hip_torque_deadzone", 0.5);
  this->declare_parameter("body_tilt_deadzone", 0.05);
  this->declare_parameter("calibration_rate", 0.01);
  this->declare_parameter("max_joint_velocity", 0.5);
  this->declare_parameter("max_position_change", 0.05);
  this->declare_parameter("ik_solver_epsilon", 0.0001);
  this->declare_parameter("enable_auto_calibration", false);
  this->declare_parameter("enable_only_when_unstable", true);

  left_leg_chain_base_ = this->get_parameter("left_leg_chain_base").as_string();
  left_leg_chain_tip_ = this->get_parameter("left_leg_chain_tip").as_string();
  right_leg_chain_base_ = this->get_parameter("right_leg_chain_base").as_string();
  right_leg_chain_tip_ = this->get_parameter("right_leg_chain_tip").as_string();
  control_rate_ = this->get_parameter("control_rate").as_double();
  ankle_torque_to_position_gain_ = this->get_parameter("ankle_torque_to_position_gain").as_double();
  hip_torque_to_position_gain_ = this->get_parameter("hip_torque_to_position_gain").as_double();
  ankle_torque_deadzone_ = this->get_parameter("ankle_torque_deadzone").as_double();
  hip_torque_deadzone_ = this->get_parameter("hip_torque_deadzone").as_double();
  body_tilt_deadzone_ = this->get_parameter("body_tilt_deadzone").as_double();
  calibration_rate_ = this->get_parameter("calibration_rate").as_double();
  max_joint_velocity_ = this->get_parameter("max_joint_velocity").as_double();
  max_position_change_ = this->get_parameter("max_position_change").as_double();
  ik_solver_epsilon_ = this->get_parameter("ik_solver_epsilon").as_double();
  enable_auto_calibration_ = this->get_parameter("enable_auto_calibration").as_bool();
  enable_only_when_unstable_ = this->get_parameter("enable_only_when_unstable").as_bool();

  // Initialize state
  current_ankle_torques_.resize(2, 0.0);
  current_hip_compensation_.resize(4, 0.0);
  current_body_tilt_.x = 0.0;
  current_body_tilt_.y = 0.0;
  current_body_tilt_.z = 0.0;

  // Create subscribers
  ankle_torque_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/balance/final_ankle_torques",
    10,
    std::bind(&LegBalanceController::ankleTorqueCallback, this, std::placeholders::_1));

  hip_compensation_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/balance/hip_compensation",
    10,
    std::bind(&LegBalanceController::hipCompensationCallback, this, std::placeholders::_1));

  body_tilt_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/balance/body_tilt",
    10,
    std::bind(&LegBalanceController::bodyTiltCallback, this, std::placeholders::_1));

  zmp_stable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/balance/zmp_stable",
    10,
    std::bind(&LegBalanceController::zmpStableCallback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&LegBalanceController::jointStateCallback, this, std::placeholders::_1));

  robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/robot_description",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&LegBalanceController::robotDescriptionCallback, this, std::placeholders::_1));

  is_getting_up_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/behavior/is_getting_up",
    10,
    std::bind(&LegBalanceController::isGettingUpCallback, this, std::placeholders::_1));

  // Create publishers
  leg_command_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/legs_controller/joint_trajectory", 10);

  calibration_offset_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/leg_balance/calibration_offset", 10);

  ik_error_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/leg_balance/ik_error", 10);

  // Create control timer
  auto control_period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
    std::bind(&LegBalanceController::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Leg Balance Controller initialized");
  RCLCPP_INFO(this->get_logger(), "  - Left leg chain: %s -> %s",
              left_leg_chain_base_.c_str(), left_leg_chain_tip_.c_str());
  RCLCPP_INFO(this->get_logger(), "  - Right leg chain: %s -> %s",
              right_leg_chain_base_.c_str(), right_leg_chain_tip_.c_str());
  RCLCPP_INFO(this->get_logger(), "  - Control rate: %.1f Hz", control_rate_);
  RCLCPP_INFO(this->get_logger(), "  - Auto-calibration: %s",
              enable_auto_calibration_ ? "enabled" : "disabled");
}

void LegBalanceController::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received robot_description, building KDL chains...");

  if (buildKDLChains(msg->data)) {
    model_loaded_ = true;
    RCLCPP_INFO(this->get_logger(), "KDL chains built successfully");
    robot_description_sub_.reset();  // Unsubscribe after receiving
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to build KDL chains");
  }
}

bool LegBalanceController::buildKDLChains(const std::string & urdf_string)
{
  // Parse URDF
  if (!robot_model_.initString(urdf_string)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
    return false;
  }

  // Build KDL tree
  if (!kdl_parser::treeFromUrdfModel(robot_model_, kdl_tree_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
    return false;
  }

  // Extract left leg chain
  if (!kdl_tree_.getChain(left_leg_chain_base_, left_leg_chain_tip_, left_leg_chain_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to extract left leg chain from %s to %s",
                 left_leg_chain_base_.c_str(), left_leg_chain_tip_.c_str());
    return false;
  }

  // Extract right leg chain
  if (!kdl_tree_.getChain(right_leg_chain_base_, right_leg_chain_tip_, right_leg_chain_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to extract right leg chain from %s to %s",
                 right_leg_chain_base_.c_str(), right_leg_chain_tip_.c_str());
    return false;
  }

  // Create FK solvers
  left_fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(left_leg_chain_);
  right_fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(right_leg_chain_);

  // Create IK solvers (Levenberg-Marquardt Algorithm)
  left_ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(left_leg_chain_);
  right_ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(right_leg_chain_);

  // Extract joint names from chains
  for (unsigned int i = 0; i < left_leg_chain_.getNrOfSegments(); ++i) {
    auto segment = left_leg_chain_.getSegment(i);
    if (segment.getJoint().getType() != KDL::Joint::None) {
      left_leg_joints_.push_back(segment.getJoint().getName());
    }
  }

  for (unsigned int i = 0; i < right_leg_chain_.getNrOfSegments(); ++i) {
    auto segment = right_leg_chain_.getSegment(i);
    if (segment.getJoint().getType() != KDL::Joint::None) {
      right_leg_joints_.push_back(segment.getJoint().getName());
    }
  }

  RCLCPP_INFO(this->get_logger(), "Left leg has %zu joints", left_leg_joints_.size());
  RCLCPP_INFO(this->get_logger(), "Right leg has %zu joints", right_leg_joints_.size());

  return true;
}

void LegBalanceController::ankleTorqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() >= 2) {
    current_ankle_torques_ = msg->data;
  }
}

void LegBalanceController::hipCompensationCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() >= 4) {
    current_hip_compensation_ = msg->data;
  }
}

void LegBalanceController::bodyTiltCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  current_body_tilt_ = msg->vector;

  // Auto-calibrate if enabled
  if (enable_auto_calibration_ && model_loaded_ && zmp_stable_) {
    updateCalibrationOffsets();
  }
}

void LegBalanceController::zmpStableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  zmp_stable_ = msg->data;
}

void LegBalanceController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  current_joint_state_ = *msg;
  joint_state_received_ = true;
}

void LegBalanceController::isGettingUpCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_getting_up_ = msg->data;
  if (is_getting_up_) {
    RCLCPP_INFO(this->get_logger(), "Get-up behavior active - leg balance controller paused");
  }
}

void LegBalanceController::controlLoop()
{
  // Wait until model is loaded and joint states received
  if (!model_loaded_ || !joint_state_received_) {
    return;
  }

  // Don't publish leg commands when get-up behavior is active
  if (is_getting_up_) {
    return;
  }

  // Check if we should apply control
  // Only apply when unstable or significant tilt detected
  if (enable_only_when_unstable_) {
    bool significant_tilt = (std::abs(current_body_tilt_.x) > body_tilt_deadzone_ ||
                            std::abs(current_body_tilt_.y) > body_tilt_deadzone_);

    if (zmp_stable_ && !significant_tilt) {
      // Robot is stable and upright, no need to apply corrections
      return;
    }
  }

  // Compute desired joint positions with balance compensations
  std::map<std::string, double> target_positions;

  // Apply ankle compensation (convert torque to position delta)
  double left_ankle_delta = ankleTorqueToPosition(current_ankle_torques_[0]);
  double right_ankle_delta = ankleTorqueToPosition(current_ankle_torques_[1]);

  // Apply hip compensation (convert torque to position delta)
  double left_hip_pitch_delta = hipTorqueToPosition(current_hip_compensation_[0]);
  double left_hip_roll_delta = hipTorqueToPosition(current_hip_compensation_[1]);
  double right_hip_pitch_delta = hipTorqueToPosition(current_hip_compensation_[2]);
  double right_hip_roll_delta = hipTorqueToPosition(current_hip_compensation_[3]);

  // Get current foot positions using forward kinematics
  KDL::JntArray left_q_current(left_leg_chain_.getNrOfJoints());
  KDL::JntArray right_q_current(right_leg_chain_.getNrOfJoints());

  // Fill current joint positions for left leg
  for (size_t i = 0; i < left_leg_joints_.size(); ++i) {
    left_q_current(i) = getJointPosition(left_leg_joints_[i]);
  }

  // Fill current joint positions for right leg
  for (size_t i = 0; i < right_leg_joints_.size(); ++i) {
    right_q_current(i) = getJointPosition(right_leg_joints_[i]);
  }

  // Calculate current foot poses
  KDL::Frame left_foot_current, right_foot_current;
  left_fk_solver_->JntToCart(left_q_current, left_foot_current);
  right_fk_solver_->JntToCart(right_q_current, right_foot_current);

  // Create target foot poses with balance compensations
  KDL::Frame left_foot_target = left_foot_current;
  KDL::Frame right_foot_target = right_foot_current;

  // Apply ankle compensation as rotation around foot center
  // Ankle pitch (sagittal plane rotation)
  left_foot_target.M = left_foot_target.M * KDL::Rotation::RotY(left_ankle_delta);
  right_foot_target.M = right_foot_target.M * KDL::Rotation::RotY(right_ankle_delta);

  // Apply hip compensation as small position adjustments
  // This shifts the foot position to compensate for hip movements
  left_foot_target.p.x(left_foot_target.p.x() + left_hip_pitch_delta * 0.1);
  left_foot_target.p.y(left_foot_target.p.y() + left_hip_roll_delta * 0.1);
  right_foot_target.p.x(right_foot_target.p.x() + right_hip_pitch_delta * 0.1);
  right_foot_target.p.y(right_foot_target.p.y() + right_hip_roll_delta * 0.1);

  // Solve IK to find joint positions that achieve target foot poses
  KDL::JntArray left_q_target(left_leg_chain_.getNrOfJoints());
  KDL::JntArray right_q_target(right_leg_chain_.getNrOfJoints());

  bool left_ik_success = computeLegIK(left_leg_chain_, *left_ik_solver_,
                                      left_foot_target, left_q_current, left_q_target);
  bool right_ik_success = computeLegIK(right_leg_chain_, *right_ik_solver_,
                                       right_foot_target, right_q_current, right_q_target);

  // Apply IK results if successful, otherwise keep current positions
  if (left_ik_success) {
    for (size_t i = 0; i < left_leg_joints_.size(); ++i) {
      double current_pos = left_q_current(i);
      double target_pos = left_q_target(i);
      double offset = calibration_offsets_[left_leg_joints_[i]];

      // Apply position change limit
      double position_change = (target_pos - current_pos) + offset;
      position_change = std::clamp(position_change, -max_position_change_, max_position_change_);

      target_positions[left_leg_joints_[i]] = current_pos + position_change;
    }
  } else {
    // IK failed, keep current positions
    for (size_t i = 0; i < left_leg_joints_.size(); ++i) {
      target_positions[left_leg_joints_[i]] = left_q_current(i);
    }
  }

  if (right_ik_success) {
    for (size_t i = 0; i < right_leg_joints_.size(); ++i) {
      double current_pos = right_q_current(i);
      double target_pos = right_q_target(i);
      double offset = calibration_offsets_[right_leg_joints_[i]];

      // Apply position change limit
      double position_change = (target_pos - current_pos) + offset;
      position_change = std::clamp(position_change, -max_position_change_, max_position_change_);

      target_positions[right_leg_joints_[i]] = current_pos + position_change;
    }
  } else {
    // IK failed, keep current positions
    for (size_t i = 0; i < right_leg_joints_.size(); ++i) {
      target_positions[right_leg_joints_[i]] = right_q_current(i);
    }
  }

  // Publish leg commands
  publishLegCommand(target_positions);
}

double LegBalanceController::ankleTorqueToPosition(double torque)
{
  // Apply dead-zone: ignore small torques
  if (std::abs(torque) < ankle_torque_deadzone_) {
    return 0.0;
  }

  // Simple proportional conversion from torque to position
  return torque * ankle_torque_to_position_gain_;
}

double LegBalanceController::hipTorqueToPosition(double torque)
{
  // Apply dead-zone: ignore small torques
  if (std::abs(torque) < hip_torque_deadzone_) {
    return 0.0;
  }

  return torque * hip_torque_to_position_gain_;
}

void LegBalanceController::updateCalibrationOffsets()
{
  // Slowly adjust calibration offsets to minimize steady-state body tilt
  // This implements integral control at a slow rate

  double tilt_threshold = 0.05;  // ~3 degrees

  // Only calibrate if tilt is persistent (not just transient)
  if (std::abs(current_body_tilt_.x) < tilt_threshold &&
      std::abs(current_body_tilt_.y) < tilt_threshold)
  {
    return;  // No significant tilt, no need to calibrate
  }

  // Update ankle offsets based on pitch (forward/backward tilt)
  for (const auto & joint_name : left_leg_joints_) {
    if (joint_name.find("ankle") != std::string::npos &&
        joint_name.find("pitch") != std::string::npos)
    {
      calibration_offsets_[joint_name] -= calibration_rate_ * current_body_tilt_.y;
    }
  }

  for (const auto & joint_name : right_leg_joints_) {
    if (joint_name.find("ankle") != std::string::npos &&
        joint_name.find("pitch") != std::string::npos)
    {
      calibration_offsets_[joint_name] -= calibration_rate_ * current_body_tilt_.y;
    }
  }

  // Publish calibration offsets for monitoring
  std_msgs::msg::Float64MultiArray offset_msg;
  for (const auto & [joint_name, offset] : calibration_offsets_) {
    offset_msg.data.push_back(offset);
  }
  calibration_offset_pub_->publish(offset_msg);
}

double LegBalanceController::getJointPosition(const std::string & joint_name)
{
  for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
    if (current_joint_state_.name[i] == joint_name) {
      return current_joint_state_.position[i];
    }
  }
  return 0.0;
}

void LegBalanceController::publishLegCommand(const std::map<std::string, double> & joint_positions)
{
  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.header.stamp = this->now();

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(1.0 / control_rate_);

  for (const auto & [joint_name, target_pos] : joint_positions) {
    traj_msg.joint_names.push_back(joint_name);
    point.positions.push_back(target_pos);

    // Compute velocity based on position change
    double current_pos = getJointPosition(joint_name);
    double vel = (target_pos - current_pos) * control_rate_;
    vel = std::clamp(vel, -max_joint_velocity_, max_joint_velocity_);
    point.velocities.push_back(vel);
  }

  traj_msg.points.push_back(point);
  leg_command_pub_->publish(traj_msg);
}

bool LegBalanceController::computeLegIK(
  const KDL::Chain & chain,
  KDL::ChainIkSolverPos_LMA & ik_solver,
  const KDL::Frame & target_pose,
  const KDL::JntArray & q_init,
  KDL::JntArray & q_out)
{
  int ret = ik_solver.CartToJnt(q_init, target_pose, q_out);
  return (ret >= 0);
}

}  // namespace balance_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(balance_control::LegBalanceController)
