#include "balance_control/get_up_behavior.hpp"
#include <cmath>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace balance_control
{

GetUpBehavior::GetUpBehavior(const rclcpp::NodeOptions & options)
: Node("get_up_behavior", options),
  current_state_(GetUpState::STANDING),
  joint_state_received_(false),
  trajectory_index_(0),
  initial_reset_duration_(1.0),
  sequences_loaded_(false)
{
  // Declare and get parameters
  this->declare_parameter("fallen_tilt_threshold", 1.0);  // ~57 degrees
  this->declare_parameter("standing_tilt_threshold", 0.3);  // ~17 degrees
  this->declare_parameter("control_rate", 50.0);
  this->declare_parameter("get_up_duration_per_point", 1.0);
  this->declare_parameter("enable_get_up", true);
  this->declare_parameter("sequences_config_file", "");

  fallen_tilt_threshold_ = this->get_parameter("fallen_tilt_threshold").as_double();
  standing_tilt_threshold_ = this->get_parameter("standing_tilt_threshold").as_double();
  control_rate_ = this->get_parameter("control_rate").as_double();
  get_up_duration_per_point_ = this->get_parameter("get_up_duration_per_point").as_double();
  enable_get_up_ = this->get_parameter("enable_get_up").as_bool();
  sequences_config_file_ = this->get_parameter("sequences_config_file").as_string();

  // Load get-up sequences from YAML file (path resolved by launch file)
  if (!sequences_config_file_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Loading get-up sequences from: %s", sequences_config_file_.c_str());

    if (loadGetUpSequences(sequences_config_file_)) {
      RCLCPP_INFO(this->get_logger(), "✓ Get-up sequences loaded successfully!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ Failed to load get-up sequences from: %s",
                   sequences_config_file_.c_str());
      RCLCPP_ERROR(this->get_logger(), "Robot will NOT be able to get up from falls!");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "No sequences_config_file parameter set!");
    RCLCPP_ERROR(this->get_logger(), "Robot will NOT be able to get up from falls!");
  }

  // Initialize body tilt
  current_body_tilt_.x = 0.0;
  current_body_tilt_.y = 0.0;
  current_body_tilt_.z = 0.0;

  // Define leg joint names
  leg_joint_names_ = {
    "dof_left_hip_yaw_03",
    "dof_left_hip_roll_03",
    "dof_left_hip_pitch_04",
    "dof_left_knee_04",
    "dof_left_ankle_02",
    "dof_right_hip_yaw_03",
    "dof_right_hip_roll_03",
    "dof_right_hip_pitch_04",
    "dof_right_knee_04",
    "dof_right_ankle_02"
  };

  // Create subscribers
  body_tilt_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/balance/body_tilt",
    10,
    std::bind(&GetUpBehavior::bodyTiltCallback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&GetUpBehavior::jointStateCallback, this, std::placeholders::_1));

  // Create publishers
  leg_command_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/legs_controller/joint_trajectory", 10);

  left_arm_command_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/left_arm_controller/joint_trajectory", 10);

  right_arm_command_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/right_arm_controller/joint_trajectory", 10);

  is_getting_up_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/behavior/is_getting_up", 10);

  is_standing_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/behavior/is_standing", 10);

  // Create control timer
  auto control_period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
    std::bind(&GetUpBehavior::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Get-Up Behavior initialized");
  RCLCPP_INFO(this->get_logger(), "  - Fallen threshold: %.2f rad (%.1f deg)",
              fallen_tilt_threshold_, fallen_tilt_threshold_ * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "  - Standing threshold: %.2f rad (%.1f deg)",
              standing_tilt_threshold_, standing_tilt_threshold_ * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "  - Control rate: %.1f Hz", control_rate_);
  RCLCPP_INFO(this->get_logger(), "  - Get-up enabled: %s", enable_get_up_ ? "true" : "false");
}

void GetUpBehavior::bodyTiltCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  current_body_tilt_ = msg->vector;
}

void GetUpBehavior::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  current_joint_state_ = *msg;
  joint_state_received_ = true;
}

void GetUpBehavior::controlLoop()
{
  if (!enable_get_up_ || !joint_state_received_) {
    return;
  }

  // State machine
  switch (current_state_) {
    case GetUpState::STANDING:
      // Publish standing status
      {
        std_msgs::msg::Bool standing_msg;
        standing_msg.data = true;
        is_standing_pub_->publish(standing_msg);

        std_msgs::msg::Bool getting_up_msg;
        getting_up_msg.data = false;
        is_getting_up_pub_->publish(getting_up_msg);
      }

      // Check if robot has fallen
      if (isFallen()) {
        RCLCPP_WARN(this->get_logger(), "Robot fallen detected! Tilt: [%.2f, %.2f, %.2f] rad",
                    current_body_tilt_.x, current_body_tilt_.y, current_body_tilt_.z);
        current_state_ = GetUpState::FALLEN;
      }
      break;

    case GetUpState::FALLEN:
      // Robot is fallen, start get-up sequence
      RCLCPP_INFO(this->get_logger(), "Starting get-up sequence...");
      startGetUpSequence();
      current_state_ = GetUpState::GETTING_UP;
      trajectory_start_time_ = this->now();
      break;

    case GetUpState::GETTING_UP:
      // Publish getting up status
      {
        std_msgs::msg::Bool standing_msg;
        standing_msg.data = false;
        is_standing_pub_->publish(standing_msg);

        std_msgs::msg::Bool getting_up_msg;
        getting_up_msg.data = true;
        is_getting_up_pub_->publish(getting_up_msg);
      }

      // Execute get-up trajectory
      executeGetUpTrajectory();

      // Check if sequence is complete
      if (trajectory_index_ >= get_up_trajectory_.size()) {
        RCLCPP_INFO(this->get_logger(), "Get-up sequence complete, stabilizing...");
        current_state_ = GetUpState::STABILIZING;
        trajectory_start_time_ = this->now();
      }
      break;

    case GetUpState::STABILIZING:
      // Wait for stabilization
      if ((this->now() - trajectory_start_time_).seconds() > 2.0) {
        if (isStanding()) {
          RCLCPP_INFO(this->get_logger(), "Robot is standing! Returning to normal operation.");
          current_state_ = GetUpState::STANDING;
        } else if (isFallen()) {
          RCLCPP_WARN(this->get_logger(), "Robot fell again during stabilization. Retrying...");
          current_state_ = GetUpState::FALLEN;
        }
      }
      break;
  }
}

bool GetUpBehavior::isFallen() const
{
  // Robot is fallen if tilt exceeds threshold in any direction
  double tilt_magnitude = std::sqrt(
    current_body_tilt_.x * current_body_tilt_.x +
    current_body_tilt_.y * current_body_tilt_.y
  );
  return tilt_magnitude > fallen_tilt_threshold_;
}

bool GetUpBehavior::isStanding() const
{
  // Robot is standing if tilt is below threshold
  double tilt_magnitude = std::sqrt(
    current_body_tilt_.x * current_body_tilt_.x +
    current_body_tilt_.y * current_body_tilt_.y
  );
  return tilt_magnitude < standing_tilt_threshold_;
}

void GetUpBehavior::startGetUpSequence()
{
  // Determine fall type based on tilt direction
  double pitch = current_body_tilt_.y;  // Side falls
  double roll = current_body_tilt_.x;   // Front/back falls

  RCLCPP_INFO(this->get_logger(), "Fall detection - Roll: %.3f rad (%.1f deg), Pitch: %.3f rad (%.1f deg)",
              roll, roll * 180.0 / M_PI, pitch, pitch * 180.0 / M_PI);

  // Check ROLL for front/back detection (robot falls around roll axis)
  // Face down: roll ≈ +π/2 (+90°)
  // On back: roll ≈ -π/2 (-90°)
  if (std::abs(roll) > 1.0) {  // Threshold: ~60 degrees
    if (roll > 0) {
      RCLCPP_INFO(this->get_logger(), "Detected fall type: FORWARD (face down, roll=+%.1f deg)",
                  roll * 180.0 / M_PI);
      get_up_trajectory_ = generateGetUpFromFront();
    } else {
      RCLCPP_INFO(this->get_logger(), "Detected fall type: BACKWARD (on back, roll=%.1f deg)",
                  roll * 180.0 / M_PI);
      get_up_trajectory_ = generateGetUpFromBack();
    }
  } else if (std::abs(pitch) > 1.0) {
    // Fallen to the side (based on pitch)
    RCLCPP_INFO(this->get_logger(), "Detected fall type: SIDE (pitch=%.1f deg)",
                pitch * 180.0 / M_PI);
    get_up_trajectory_ = generateGetUpFromSide();
  } else {
    // Unclear - default to back
    RCLCPP_WARN(this->get_logger(), "Unclear fall orientation (roll=%.1f, pitch=%.1f deg), using BACK sequence",
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI);
    get_up_trajectory_ = generateGetUpFromBack();
  }

  trajectory_index_ = 0;
  get_up_cumulative_times_.clear();
  double accumulated = 0.0;
  for (const auto & step : get_up_trajectory_) {
    double step_duration = step.duration > 0.0 ? step.duration : get_up_duration_per_point_;
    accumulated += step_duration;
    get_up_cumulative_times_.push_back(accumulated);
  }
}

void GetUpBehavior::executeGetUpTrajectory()
{
  if (get_up_trajectory_.empty()) {
    return;
  }

  double elapsed = (this->now() - trajectory_start_time_).seconds();

  while (trajectory_index_ < get_up_trajectory_.size() &&
         trajectory_index_ < get_up_cumulative_times_.size() &&
         elapsed >= get_up_cumulative_times_[trajectory_index_]) {
    const auto & step = get_up_trajectory_[trajectory_index_];
    double step_duration = step.duration > 0.0 ? step.duration : get_up_duration_per_point_;
    RCLCPP_INFO(this->get_logger(), "Executing trajectory point %zu/%zu (duration %.2f s)",
                trajectory_index_ + 1, get_up_trajectory_.size(), step_duration);
    publishTrajectoryPoint(step.positions, step_duration);
    trajectory_index_++;
  }
}

void GetUpBehavior::publishTrajectoryPoint(
  const std::map<std::string, double> & positions,
  double duration)
{
  // Define joint order as expected by controllers (MUST match controllers.yaml)
  const std::vector<std::string> leg_joint_order = {
    "dof_left_hip_pitch_04",
    "dof_left_hip_roll_03",
    "dof_left_hip_yaw_03",
    "dof_left_knee_04",
    "dof_left_ankle_02",
    "dof_right_hip_pitch_04",
    "dof_right_hip_roll_03",
    "dof_right_hip_yaw_03",
    "dof_right_knee_04",
    "dof_right_ankle_02"
  };

  const std::vector<std::string> left_arm_joint_order = {
    "dof_left_shoulder_pitch_03",
    "dof_left_shoulder_roll_03",
    "dof_left_shoulder_yaw_02",
    "dof_left_elbow_02",
    "dof_left_wrist_00"
  };

  const std::vector<std::string> right_arm_joint_order = {
    "dof_right_shoulder_pitch_03",
    "dof_right_shoulder_roll_03",
    "dof_right_shoulder_yaw_02",
    "dof_right_elbow_02",
    "dof_right_wrist_00"
  };

  trajectory_msgs::msg::JointTrajectory leg_traj, left_arm_traj, right_arm_traj;
  trajectory_msgs::msg::JointTrajectoryPoint leg_point, left_arm_point, right_arm_point;

  leg_traj.header.stamp = this->now();
  left_arm_traj.header.stamp = this->now();
  right_arm_traj.header.stamp = this->now();

  leg_point.time_from_start = rclcpp::Duration::from_seconds(duration);
  left_arm_point.time_from_start = rclcpp::Duration::from_seconds(duration);
  right_arm_point.time_from_start = rclcpp::Duration::from_seconds(duration);

  // Build leg trajectory in correct order
  for (const auto & joint_name : leg_joint_order) {
    auto it = positions.find(joint_name);
    if (it != positions.end()) {
      leg_traj.joint_names.push_back(joint_name);
      leg_point.positions.push_back(it->second);
      leg_point.velocities.push_back(0.0);
    }
  }

  // Build left arm trajectory in correct order
  for (const auto & joint_name : left_arm_joint_order) {
    auto it = positions.find(joint_name);
    if (it != positions.end()) {
      left_arm_traj.joint_names.push_back(joint_name);
      left_arm_point.positions.push_back(it->second);
      left_arm_point.velocities.push_back(0.0);
    }
  }

  // Build right arm trajectory in correct order
  for (const auto & joint_name : right_arm_joint_order) {
    auto it = positions.find(joint_name);
    if (it != positions.end()) {
      right_arm_traj.joint_names.push_back(joint_name);
      right_arm_point.positions.push_back(it->second);
      right_arm_point.velocities.push_back(0.0);
    }
  }

  // Publish leg commands
  if (!leg_traj.joint_names.empty()) {
    leg_traj.points.push_back(leg_point);
    leg_command_pub_->publish(leg_traj);
  }

  // Publish left arm commands
  if (!left_arm_traj.joint_names.empty()) {
    left_arm_traj.points.push_back(left_arm_point);
    left_arm_command_pub_->publish(left_arm_traj);
  }

  // Publish right arm commands
  if (!right_arm_traj.joint_names.empty()) {
    right_arm_traj.points.push_back(right_arm_point);
    right_arm_command_pub_->publish(right_arm_traj);
  }
}

std::map<std::string, double> GetUpBehavior::getCurrentJointPositions()
{
  std::map<std::string, double> positions;
  for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
    positions[current_joint_state_.name[i]] = current_joint_state_.position[i];
  }
  return positions;
}

std::vector<GetUpBehavior::TrajectoryStep> GetUpBehavior::generateGetUpFromBack()
{
  // Return YAML-configured sequence if available
  if (sequences_loaded_ && !back_sequence_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Using YAML-configured back sequence (%zu steps)",
                back_sequence_.size());
    return back_sequence_;
  }

  // No sequence available - log error and return empty
  RCLCPP_ERROR(this->get_logger(),
               "No get-up sequence loaded! Please configure sequences_config_file parameter.");
  return std::vector<TrajectoryStep>();
}

std::vector<GetUpBehavior::TrajectoryStep> GetUpBehavior::generateGetUpFromFront()
{
  // Return YAML-configured sequence if available
  if (sequences_loaded_ && !front_sequence_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Using YAML-configured front sequence (%zu steps)",
                front_sequence_.size());
    return front_sequence_;
  }

  // No sequence available - log error and return empty
  RCLCPP_ERROR(this->get_logger(),
               "No get-up sequence loaded! Please configure sequences_config_file parameter.");
  return std::vector<TrajectoryStep>();
}

bool GetUpBehavior::loadGetUpSequences(const std::string & yaml_file_path)
{
  try {
    YAML::Node config = YAML::LoadFile(yaml_file_path);

    // Load initial reset positions and duration (fallback to parameter)
    initial_reset_duration_ = get_up_duration_per_point_;
    if (config["initial_reset"]) {
      if (config["initial_reset"]["positions"]) {
        YAML::Node positions = config["initial_reset"]["positions"];
        for (YAML::const_iterator it = positions.begin(); it != positions.end(); ++it) {
          std::string joint_name = it->first.as<std::string>();
          double position = it->second.as<double>();
          initial_reset_positions_[joint_name] = position;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu initial reset positions", initial_reset_positions_.size());
      }
      if (config["initial_reset"]["duration"]) {
        initial_reset_duration_ = config["initial_reset"]["duration"].as<double>();
      }
    }

    // Load get-up from front sequence
    if (config["get_up_from_front"]) {
      front_sequence_ = parseSequenceFromYAML(config["get_up_from_front"]);
      RCLCPP_INFO(this->get_logger(), "Loaded front sequence with %zu steps", front_sequence_.size());
    }

    // Load get-up from back sequence
    if (config["get_up_from_back"]) {
      back_sequence_ = parseSequenceFromYAML(config["get_up_from_back"]);
      RCLCPP_INFO(this->get_logger(), "Loaded back sequence with %zu steps", back_sequence_.size());
    }

    sequences_loaded_ = true;
    return true;

  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading sequences: %s", e.what());
    return false;
  }
}

std::vector<GetUpBehavior::TrajectoryStep> GetUpBehavior::parseSequenceFromYAML(const YAML::Node & sequence_node)
{
  std::vector<TrajectoryStep> sequence;

  // Always start with initial reset (all joints to 0)
  if (!initial_reset_positions_.empty()) {
    sequence.push_back({initial_reset_positions_, initial_reset_duration_});
    RCLCPP_INFO(this->get_logger(), "Added initial reset step to sequence (%.2f s)", initial_reset_duration_);
  }

  // Parse each step in the sequence
  for (size_t i = 1; i <= 10; ++i) {  // Support up to 10 steps
    std::string step_name = "step_" + std::to_string(i);
    if (sequence_node[step_name] && sequence_node[step_name]["positions"]) {
      std::map<std::string, double> step_positions;
      YAML::Node positions = sequence_node[step_name]["positions"];

      for (YAML::const_iterator it = positions.begin(); it != positions.end(); ++it) {
        std::string joint_name = it->first.as<std::string>();
        double position = it->second.as<double>();
        step_positions[joint_name] = position;
      }

      if (!step_positions.empty()) {
        double duration = get_up_duration_per_point_;
        if (sequence_node[step_name]["duration"]) {
          duration = sequence_node[step_name]["duration"].as<double>();
        }
        sequence.push_back({step_positions, duration});
      }
    }
  }

  return sequence;
}

std::vector<GetUpBehavior::TrajectoryStep> GetUpBehavior::generateGetUpFromSide()
{
  // Get-up sequence from side (similar to back, adjusted for side orientation)
  // For simplicity, use the same as back get-up
  return generateGetUpFromBack();
}

}  // namespace balance_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(balance_control::GetUpBehavior)
