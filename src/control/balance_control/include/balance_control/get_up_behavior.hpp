#ifndef BALANCE_CONTROL__GET_UP_BEHAVIOR_HPP_
#define BALANCE_CONTROL__GET_UP_BEHAVIOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>
#include <map>
#include <vector>
#include <string>

namespace balance_control
{

/**
 * @brief Behavior node that detects when robot is fallen and executes recovery motion
 *
 * This node monitors body tilt and triggers a get-up sequence when the robot is fallen.
 * It publishes joint trajectories to move the robot from fallen to standing position.
 */
class GetUpBehavior : public rclcpp::Node
{
public:
  explicit GetUpBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callback functions
  void bodyTiltCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void controlLoop();

  // Get-up state machine
  enum class GetUpState {
    STANDING,      // Robot is upright, no action needed
    FALLEN,        // Robot is fallen, ready to start get-up
    GETTING_UP,    // Executing get-up sequence
    STABILIZING    // Final stabilization before returning to standing
  };

  // Helper functions
  bool isFallen() const;
  bool isStanding() const;
  void startGetUpSequence();
  void executeGetUpTrajectory();
  void publishTrajectoryPoint(const std::map<std::string, double> & positions, double duration);
  std::map<std::string, double> getCurrentJointPositions();

  // Load get-up sequences from YAML configuration file
  bool loadGetUpSequences(const std::string & yaml_file_path);

  // Parse a sequence from YAML node
  struct TrajectoryStep {
    std::map<std::string, double> positions;
    double duration;  // seconds
  };
  std::vector<TrajectoryStep> parseSequenceFromYAML(const YAML::Node & sequence_node);

  // Trajectory generation for different fall types (now loads from YAML)
  std::vector<TrajectoryStep> generateGetUpFromBack();
  std::vector<TrajectoryStep> generateGetUpFromFront();
  std::vector<TrajectoryStep> generateGetUpFromSide();

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr body_tilt_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Publishers
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr leg_command_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_arm_command_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_arm_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_getting_up_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_standing_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State variables
  GetUpState current_state_;
  geometry_msgs::msg::Vector3 current_body_tilt_;
  sensor_msgs::msg::JointState current_joint_state_;
  bool joint_state_received_;

  // Get-up sequence
  std::vector<TrajectoryStep> get_up_trajectory_;
  std::vector<double> get_up_cumulative_times_;
  size_t trajectory_index_;
  rclcpp::Time trajectory_start_time_;

  // Loaded sequences from YAML
  std::map<std::string, double> initial_reset_positions_;
  std::vector<TrajectoryStep> front_sequence_;
  std::vector<TrajectoryStep> back_sequence_;
  double initial_reset_duration_;
  bool sequences_loaded_;

  // Parameters
  std::string sequences_config_file_;
  double fallen_tilt_threshold_;      // Tilt threshold to detect fallen (radians)
  double standing_tilt_threshold_;    // Tilt threshold to confirm standing (radians)
  double control_rate_;               // Control loop rate (Hz)
  double get_up_duration_per_point_;  // Time for each trajectory point (seconds)
  bool enable_get_up_;                // Enable/disable get-up behavior

  // Joint names
  std::vector<std::string> leg_joint_names_;
};

}  // namespace balance_control

#endif  // BALANCE_CONTROL__GET_UP_BEHAVIOR_HPP_
