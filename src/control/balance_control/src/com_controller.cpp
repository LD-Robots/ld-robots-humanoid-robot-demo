#include "balance_control/com_controller.hpp"
#include <cmath>
#include <algorithm>

namespace balance_control
{

CoMController::CoMController(const rclcpp::NodeOptions & options)
: Node("com_controller", options),
  joint_state_received_(false),
  body_tilt_received_(false)
{
  // Declare and get parameters
  this->declare_parameter("base_link", "base_link");
  this->declare_parameter("com_kp", 1.0);
  this->declare_parameter("com_kd", 0.1);
  this->declare_parameter("ankle_weight", 0.7);
  this->declare_parameter("hip_weight", 0.3);

  base_link_ = this->get_parameter("base_link").as_string();
  com_kp_ = this->get_parameter("com_kp").as_double();
  com_kd_ = this->get_parameter("com_kd").as_double();
  ankle_weight_ = this->get_parameter("ankle_weight").as_double();
  hip_weight_ = this->get_parameter("hip_weight").as_double();

  // Create subscribers
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&CoMController::jointStateCallback, this, std::placeholders::_1));

  body_tilt_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/balance/body_tilt",
    10,
    std::bind(&CoMController::bodyTiltCallback, this, std::placeholders::_1));

  // Subscribe to robot_description topic to get URDF (transient_local for latched behavior)
  robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/robot_description",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&CoMController::robotDescriptionCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Waiting for robot_description on /robot_description topic...");

  // Create publishers
  com_position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/balance/com_position", 10);

  com_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/balance/com_velocity", 10);

  com_error_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/balance/com_error", 10);

  ankle_compensation_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/balance/ankle_compensation", 10);

  hip_compensation_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/balance/hip_compensation", 10);

  // Initialize previous CoM position
  previous_com_position_.x = 0.0;
  previous_com_position_.y = 0.0;
  previous_com_position_.z = 0.0;
  previous_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "CoM Controller initialized");
  RCLCPP_INFO(this->get_logger(), "  - Base link: %s", base_link_.c_str());
  RCLCPP_INFO(this->get_logger(), "  - CoM Kp: %.2f, Kd: %.2f", com_kp_, com_kd_);
  RCLCPP_INFO(this->get_logger(), "  - Ankle weight: %.2f, Hip weight: %.2f",
              ankle_weight_, hip_weight_);
}

void CoMController::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received robot_description, loading model...");

  // Parse URDF
  if (!robot_model_.initString(msg->data)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF - using simplified model");
    return;
  }

  // Build KDL tree
  if (!kdl_parser::treeFromUrdfModel(robot_model_, kdl_tree_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree - using simplified model");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Robot model loaded successfully from /robot_description topic");

  // Unsubscribe after receiving (we only need it once)
  robot_description_sub_.reset();
}

bool CoMController::loadRobotModel()
{
  // This function is kept for compatibility but robot model is now loaded
  // via topic subscription in robotDescriptionCallback
  return true;
}

void CoMController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  current_joint_state_ = *msg;
  joint_state_received_ = true;

  // Only proceed if we have body tilt information
  if (!body_tilt_received_) {
    return;
  }

  // Calculate current CoM position
  auto com_position = calculateCoMPosition();

  // Calculate current CoM velocity
  auto com_velocity = calculateCoMVelocity();

  // Calculate desired CoM position based on body tilt
  auto desired_com = calculateDesiredCoM(current_body_tilt_);

  // Calculate CoM error
  geometry_msgs::msg::Vector3 com_error;
  com_error.x = desired_com.x - com_position.x;
  com_error.y = desired_com.y - com_position.y;
  com_error.z = desired_com.z - com_position.z;

  // Compute compensation torques
  auto ankle_comp = computeAnkleCompensation(com_error, com_velocity);
  auto hip_comp = computeHipCompensation(com_error, com_velocity);

  // Publish CoM position
  geometry_msgs::msg::PointStamped com_pos_msg;
  com_pos_msg.header.stamp = this->now();
  com_pos_msg.header.frame_id = base_link_;
  com_pos_msg.point = com_position;
  com_position_pub_->publish(com_pos_msg);

  // Publish CoM velocity
  geometry_msgs::msg::Vector3Stamped com_vel_msg;
  com_vel_msg.header.stamp = this->now();
  com_vel_msg.header.frame_id = base_link_;
  com_vel_msg.vector = com_velocity;
  com_velocity_pub_->publish(com_vel_msg);

  // Publish CoM error
  geometry_msgs::msg::Vector3Stamped com_error_msg;
  com_error_msg.header.stamp = this->now();
  com_error_msg.header.frame_id = base_link_;
  com_error_msg.vector = com_error;
  com_error_pub_->publish(com_error_msg);

  // Publish ankle compensation
  std_msgs::msg::Float64MultiArray ankle_msg;
  ankle_msg.data = ankle_comp;
  ankle_compensation_pub_->publish(ankle_msg);

  // Publish hip compensation
  std_msgs::msg::Float64MultiArray hip_msg;
  hip_msg.data = hip_comp;
  hip_compensation_pub_->publish(hip_msg);

  // Update previous CoM position
  previous_com_position_ = com_position;
  previous_time_ = this->now();
}

void CoMController::bodyTiltCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  current_body_tilt_ = msg->vector;
  body_tilt_received_ = true;
}

geometry_msgs::msg::Point CoMController::calculateCoMPosition()
{
  // Simplified CoM calculation
  // In a full implementation, this would use forward kinematics for all links
  // and compute the weighted average based on link masses

  geometry_msgs::msg::Point com;

  // For now, estimate CoM based on joint positions
  // Assume CoM is approximately at the torso center with small offsets based on joint angles
  com.x = 0.0;
  com.y = 0.0;
  com.z = 0.5;  // Approximate height of CoM for a humanoid

  // Add small adjustments based on hip and torso angles
  for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
    const auto & joint_name = current_joint_state_.name[i];
    double position = current_joint_state_.position[i];

    // Hip pitch affects CoM in X direction
    if (joint_name.find("hip_pitch") != std::string::npos) {
      com.x += position * 0.05;  // Small contribution
    }

    // Hip roll affects CoM in Y direction
    if (joint_name.find("hip_roll") != std::string::npos) {
      com.y += position * 0.05;
    }
  }

  return com;
}

geometry_msgs::msg::Vector3 CoMController::calculateCoMVelocity()
{
  geometry_msgs::msg::Vector3 velocity;

  // Calculate velocity using finite differences
  auto current_time = this->now();
  double dt = (current_time - previous_time_).seconds();

  if (dt > 0.0 && dt < 1.0) {  // Sanity check
    auto current_com = calculateCoMPosition();
    velocity.x = (current_com.x - previous_com_position_.x) / dt;
    velocity.y = (current_com.y - previous_com_position_.y) / dt;
    velocity.z = (current_com.z - previous_com_position_.z) / dt;
  } else {
    velocity.x = 0.0;
    velocity.y = 0.0;
    velocity.z = 0.0;
  }

  return velocity;
}

geometry_msgs::msg::Point CoMController::calculateDesiredCoM(
  const geometry_msgs::msg::Vector3 & body_tilt)
{
  geometry_msgs::msg::Point desired_com;

  // Desired CoM should be shifted to counteract body tilt
  // If robot tilts forward (positive pitch), shift CoM backward (negative X)
  desired_com.x = -body_tilt.y * 0.1;  // Pitch compensation
  desired_com.y = -body_tilt.x * 0.1;  // Roll compensation
  desired_com.z = 0.5;  // Maintain nominal height

  return desired_com;
}

std::vector<double> CoMController::computeAnkleCompensation(
  const geometry_msgs::msg::Vector3 & com_error,
  const geometry_msgs::msg::Vector3 & com_velocity)
{
  // PD controller for ankle compensation
  // ankle_torque = Kp * error + Kd * velocity

  std::vector<double> ankle_comp(2, 0.0);  // [left_ankle, right_ankle]

  // Ankle pitch compensation (sagittal plane - forward/backward)
  double pitch_compensation = com_kp_ * com_error.x + com_kd_ * com_velocity.x;
  pitch_compensation *= ankle_weight_;

  // Both ankles contribute equally to pitch correction
  ankle_comp[0] = pitch_compensation;  // Left ankle
  ankle_comp[1] = pitch_compensation;  // Right ankle

  // Clamp to safe limits
  double max_ankle_torque = 10.0;  // Nm
  for (auto & torque : ankle_comp) {
    torque = std::clamp(torque, -max_ankle_torque, max_ankle_torque);
  }

  return ankle_comp;
}

std::vector<double> CoMController::computeHipCompensation(
  const geometry_msgs::msg::Vector3 & com_error,
  const geometry_msgs::msg::Vector3 & com_velocity)
{
  // PD controller for hip compensation
  std::vector<double> hip_comp(4, 0.0);  // [left_hip_pitch, left_hip_roll, right_hip_pitch, right_hip_roll]

  // Hip pitch compensation
  double pitch_compensation = com_kp_ * com_error.x + com_kd_ * com_velocity.x;
  pitch_compensation *= hip_weight_;

  hip_comp[0] = -pitch_compensation;  // Left hip pitch (opposite direction to ankle)
  hip_comp[2] = -pitch_compensation;  // Right hip pitch

  // Hip roll compensation
  double roll_compensation = com_kp_ * com_error.y + com_kd_ * com_velocity.y;
  roll_compensation *= hip_weight_;

  hip_comp[1] = roll_compensation;   // Left hip roll
  hip_comp[3] = -roll_compensation;  // Right hip roll (opposite for balance)

  // Clamp to safe limits
  double max_hip_torque = 15.0;  // Nm
  for (auto & torque : hip_comp) {
    torque = std::clamp(torque, -max_hip_torque, max_hip_torque);
  }

  return hip_comp;
}

}  // namespace balance_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(balance_control::CoMController)
