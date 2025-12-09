#ifndef BALANCE_CONTROL__LEG_BALANCE_CONTROLLER_HPP_
#define BALANCE_CONTROL__LEG_BALANCE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <urdf/model.h>
#include <memory>
#include <vector>
#include <string>
#include <map>

namespace balance_control
{

/**
 * @brief Leg Balance Controller Node
 *
 * This node controls the robot's legs to maintain balance by applying compensations
 * calculated by the balance control system. It uses KDL (Kinematics and Dynamics Library)
 * for inverse kinematics and auto-calibrates joint positions based on IMU feedback.
 *
 * The controller receives balance compensations (ankle and hip torques) and translates
 * them into joint position commands using inverse kinematics. It continuously adjusts
 * joint offsets to minimize body tilt and maintain stability.
 *
 * Subscribed Topics:
 *   - /balance/final_ankle_torques (std_msgs/msg/Float64MultiArray): Final ankle torque commands
 *   - /balance/hip_compensation (std_msgs/msg/Float64MultiArray): Hip joint corrections
 *   - /balance/body_tilt (geometry_msgs/msg/Vector3Stamped): Body orientation (roll, pitch, yaw)
 *   - /balance/zmp_stable (std_msgs/msg/Bool): ZMP stability status
 *   - /joint_states (sensor_msgs/msg/JointState): Current joint states
 *   - /robot_description (std_msgs/msg/String): Robot URDF model
 *
 * Published Topics:
 *   - /legs_controller/joint_trajectory (trajectory_msgs/msg/JointTrajectory): Leg joint commands
 *   - /leg_balance/calibration_offset (std_msgs/msg/Float64MultiArray): Auto-calibration offsets
 *   - /leg_balance/ik_error (geometry_msgs/msg/Vector3Stamped): IK solver error
 *
 * Parameters:
 *   - left_leg_chain_base (string): Base link for left leg chain, default: "base"
 *   - left_leg_chain_tip (string): Tip link for left leg chain, default: "LFootBushing_GPF_1517_12"
 *   - right_leg_chain_base (string): Base link for right leg chain, default: "base"
 *   - right_leg_chain_tip (string): Tip link for right leg chain, default: "RFootBushing_GPF_1517_12"
 *   - control_rate (double): Control loop rate in Hz, default: 100.0
 *   - ankle_torque_to_position_gain (double): Conversion gain from torque to position, default: 0.01
 *   - hip_torque_to_position_gain (double): Conversion gain from torque to position, default: 0.01
 *   - calibration_rate (double): Auto-calibration update rate, default: 0.1
 *   - max_joint_velocity (double): Maximum joint velocity (rad/s), default: 1.0
 *   - ik_solver_epsilon (double): IK solver convergence threshold, default: 1e-4
 *   - enable_auto_calibration (bool): Enable automatic calibration, default: true
 */
class LegBalanceController : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options for ROS 2 configuration
   */
  explicit LegBalanceController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~LegBalanceController() = default;

private:
  /**
   * @brief Main control loop timer callback
   * Executes at control_rate Hz to compute and publish leg joint commands
   */
  void controlLoop();

  /**
   * @brief Callback for ankle torque commands from balance control
   * @param msg Ankle torque array [left_ankle, right_ankle]
   */
  void ankleTorqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  /**
   * @brief Callback for hip compensation from balance control
   * @param msg Hip compensation array [left_hip_pitch, left_hip_roll, right_hip_pitch, right_hip_roll]
   */
  void hipCompensationCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  /**
   * @brief Callback for body tilt from IMU
   * @param msg Body tilt angles (roll, pitch, yaw)
   */
  void bodyTiltCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  /**
   * @brief Callback for ZMP stability status
   * @param msg Boolean indicating if ZMP is within support polygon
   */
  void zmpStableCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Callback for joint states
   * @param msg Current joint positions, velocities, efforts
   */
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief Callback for robot description (URDF)
   * @param msg String containing robot URDF
   */
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Load robot model and build KDL chains for legs
   * @param urdf_string URDF model as string
   * @return true if successful
   */
  bool buildKDLChains(const std::string & urdf_string);

  /**
   * @brief Convert ankle torque compensation to ankle joint position delta
   * @param torque Ankle torque command (Nm)
   * @return Position delta (radians)
   */
  double ankleTorqueToPosition(double torque);

  /**
   * @brief Convert hip torque compensation to hip joint position delta
   * @param torque Hip torque command (Nm)
   * @return Position delta (radians)
   */
  double hipTorqueToPosition(double torque);

  /**
   * @brief Compute inverse kinematics for a leg
   * @param chain KDL chain representing the leg
   * @param ik_solver IK solver for the chain
   * @param target_pose Desired end-effector pose
   * @param q_init Initial joint configuration
   * @param q_out Output joint configuration
   * @return true if IK converged
   */
  bool computeLegIK(
    const KDL::Chain & chain,
    KDL::ChainIkSolverPos_LMA & ik_solver,
    const KDL::Frame & target_pose,
    const KDL::JntArray & q_init,
    KDL::JntArray & q_out);

  /**
   * @brief Auto-calibrate joint offsets based on body tilt
   * Slowly adjusts joint offsets to minimize steady-state tilt error
   */
  void updateCalibrationOffsets();

  /**
   * @brief Get current joint position for a specific joint
   * @param joint_name Name of the joint
   * @return Joint position (radians), or 0.0 if not found
   */
  double getJointPosition(const std::string & joint_name);

  /**
   * @brief Publish leg joint trajectory command
   * @param joint_positions Map of joint names to target positions
   */
  void publishLegCommand(const std::map<std::string, double> & joint_positions);

  // ROS 2 interfaces
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ankle_torque_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr hip_compensation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr body_tilt_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr zmp_stable_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr leg_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr calibration_offset_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr ik_error_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  // KDL structures
  urdf::Model robot_model_;
  KDL::Tree kdl_tree_;
  KDL::Chain left_leg_chain_;
  KDL::Chain right_leg_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> left_fk_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> right_fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> left_ik_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> right_ik_solver_;

  // State variables
  sensor_msgs::msg::JointState current_joint_state_;
  std::vector<double> current_ankle_torques_;     // [left, right]
  std::vector<double> current_hip_compensation_;  // [left_pitch, left_roll, right_pitch, right_roll]
  geometry_msgs::msg::Vector3 current_body_tilt_;
  bool zmp_stable_;
  bool model_loaded_;
  bool joint_state_received_;

  // Calibration offsets (learned over time)
  std::map<std::string, double> calibration_offsets_;

  // Leg joint names (populated after loading URDF)
  std::vector<std::string> left_leg_joints_;
  std::vector<std::string> right_leg_joints_;

  // Parameters
  std::string left_leg_chain_base_;
  std::string left_leg_chain_tip_;
  std::string right_leg_chain_base_;
  std::string right_leg_chain_tip_;
  double control_rate_;
  double ankle_torque_to_position_gain_;
  double hip_torque_to_position_gain_;
  double ankle_torque_deadzone_;
  double hip_torque_deadzone_;
  double body_tilt_deadzone_;
  double calibration_rate_;
  double max_joint_velocity_;
  double max_position_change_;
  double ik_solver_epsilon_;
  bool enable_auto_calibration_;
  bool enable_only_when_unstable_;
};

}  // namespace balance_control

#endif  // BALANCE_CONTROL__LEG_BALANCE_CONTROLLER_HPP_
