#ifndef BALANCE_CONTROL__COM_CONTROLLER_HPP_
#define BALANCE_CONTROL__COM_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <memory>
#include <string>
#include <vector>

namespace balance_control
{

/**
 * @brief Center of Mass (CoM) Controller Node
 *
 * This node computes the robot's center of mass position and velocity based on
 * joint states and the URDF model. It provides CoM feedback for balance control
 * and can command CoM adjustments through leg joint modifications.
 *
 * The CoM controller uses forward kinematics to calculate the weighted average
 * position of all body segments and generates compensating ankle/hip torques
 * to maintain the CoM above the support polygon.
 *
 * Subscribed Topics:
 *   - /joint_states (sensor_msgs/msg/JointState): Current joint positions and velocities
 *   - /balance/body_tilt (geometry_msgs/msg/Vector3Stamped): Body tilt angles from IMU
 *
 * Published Topics:
 *   - /balance/com_position (geometry_msgs/msg/PointStamped): Current CoM position
 *   - /balance/com_velocity (geometry_msgs/msg/Vector3Stamped): CoM velocity
 *   - /balance/com_error (geometry_msgs/msg/Vector3Stamped): CoM error from desired position
 *   - /balance/ankle_compensation (std_msgs/msg/Float64MultiArray): Ankle joint corrections
 *   - /balance/hip_compensation (std_msgs/msg/Float64MultiArray): Hip joint corrections
 *
 * Parameters:
 *   - robot_description (string): URDF model of the robot
 *   - base_link (string): Name of the robot base link, default: "base_link"
 *   - com_kp (double): CoM proportional gain, default: 1.0
 *   - com_kd (double): CoM derivative gain, default: 0.1
 *   - ankle_weight (double): Ankle compensation weight (0-1), default: 0.7
 *   - hip_weight (double): Hip compensation weight (0-1), default: 0.3
 */
class CoMController : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options for ROS 2 configuration
   */
  explicit CoMController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~CoMController() = default;

private:
  /**
   * @brief Callback for joint state updates
   * @param msg Joint state message with positions, velocities, efforts
   */
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief Callback for body tilt updates from IMU processor
   * @param msg Body tilt angles (roll, pitch, yaw)
   */
  void bodyTiltCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  /**
   * @brief Calculate the center of mass position using forward kinematics
   * @return CoM position in base_link frame
   */
  geometry_msgs::msg::Point calculateCoMPosition();

  /**
   * @brief Calculate the center of mass velocity using joint velocities
   * @return CoM velocity vector
   */
  geometry_msgs::msg::Vector3 calculateCoMVelocity();

  /**
   * @brief Calculate desired CoM position to maintain balance
   * @param body_tilt Current body tilt angles
   * @return Desired CoM position
   */
  geometry_msgs::msg::Point calculateDesiredCoM(const geometry_msgs::msg::Vector3 & body_tilt);

  /**
   * @brief Compute ankle compensation torques based on CoM error
   * @param com_error CoM position error
   * @param com_velocity CoM velocity
   * @return Array of ankle joint compensation values [left_ankle, right_ankle]
   */
  std::vector<double> computeAnkleCompensation(
    const geometry_msgs::msg::Vector3 & com_error,
    const geometry_msgs::msg::Vector3 & com_velocity);

  /**
   * @brief Compute hip compensation torques based on CoM error
   * @param com_error CoM position error
   * @param com_velocity CoM velocity
   * @return Array of hip joint compensation values [left_hip_pitch, right_hip_pitch, ...]
   */
  std::vector<double> computeHipCompensation(
    const geometry_msgs::msg::Vector3 & com_error,
    const geometry_msgs::msg::Vector3 & com_velocity);

  /**
   * @brief Load robot URDF model and initialize KDL structures
   * @return true if successful
   */
  bool loadRobotModel();

  /**
   * @brief Callback for robot_description topic
   * @param msg String message containing URDF
   */
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);

  // ROS 2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr body_tilt_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr com_position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr com_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr com_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ankle_compensation_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr hip_compensation_pub_;

  // Robot model
  urdf::Model robot_model_;
  KDL::Tree kdl_tree_;
  std::vector<KDL::Chain> kdl_chains_;

  // State variables
  sensor_msgs::msg::JointState current_joint_state_;
  geometry_msgs::msg::Vector3 current_body_tilt_;
  geometry_msgs::msg::Point previous_com_position_;
  rclcpp::Time previous_time_;
  bool joint_state_received_;
  bool body_tilt_received_;

  // Parameters
  std::string base_link_;
  double com_kp_;
  double com_kd_;
  double ankle_weight_;
  double hip_weight_;
};

}  // namespace balance_control

#endif  // BALANCE_CONTROL__COM_CONTROLLER_HPP_
