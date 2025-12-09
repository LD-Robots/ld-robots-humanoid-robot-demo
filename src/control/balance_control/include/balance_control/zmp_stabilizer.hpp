#ifndef BALANCE_CONTROL__ZMP_STABILIZER_HPP_
#define BALANCE_CONTROL__ZMP_STABILIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <memory>

namespace balance_control
{

/**
 * @brief Zero Moment Point (ZMP) Stabilizer Node
 *
 * This node implements ZMP-based balance control for bipedal robots. The ZMP is the point
 * on the ground where the total moment of ground reaction forces is zero. For stable
 * standing/walking, the ZMP must remain within the support polygon (convex hull of
 * contact points with the ground).
 *
 * The stabilizer monitors the ZMP position, compares it with the support polygon,
 * and generates corrective ankle torques to keep the ZMP inside the stable region.
 *
 * Subscribed Topics:
 *   - /balance/com_position (geometry_msgs/msg/PointStamped): Center of mass position
 *   - /balance/com_velocity (geometry_msgs/msg/Vector3Stamped): CoM velocity
 *   - /balance/body_tilt (geometry_msgs/msg/Vector3Stamped): Body tilt from IMU
 *   - /balance/ankle_compensation (std_msgs/msg/Float64MultiArray): Ankle corrections from CoM controller
 *
 * Published Topics:
 *   - /balance/zmp_position (geometry_msgs/msg/PointStamped): Current ZMP position
 *   - /balance/support_polygon (geometry_msgs/msg/PolygonStamped): Current support polygon
 *   - /balance/zmp_stable (std_msgs/msg/Bool): True if ZMP is within support polygon
 *   - /balance/final_ankle_torques (std_msgs/msg/Float64MultiArray): Final ankle torque commands
 *   - /balance/zmp_error (geometry_msgs/msg/Vector3Stamped): ZMP distance from polygon center
 *
 * Parameters:
 *   - foot_length (double): Length of robot foot (m), default: 0.2
 *   - foot_width (double): Width of robot foot (m), default: 0.1
 *   - zmp_kp (double): ZMP proportional gain, default: 2.0
 *   - zmp_kd (double): ZMP derivative gain, default: 0.5
 *   - safety_margin (double): Safety margin from polygon edge (m), default: 0.02
 *   - max_zmp_torque (double): Maximum ZMP correction torque (Nm), default: 20.0
 */
class ZMPStabilizer : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options for ROS 2 configuration
   */
  explicit ZMPStabilizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~ZMPStabilizer() = default;

private:
  /**
   * @brief Callback for CoM position updates
   * @param msg CoM position in base frame
   */
  void comPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  /**
   * @brief Callback for CoM velocity updates
   * @param msg CoM velocity vector
   */
  void comVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  /**
   * @brief Callback for body tilt updates
   * @param msg Body tilt angles (roll, pitch, yaw)
   */
  void bodyTiltCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  /**
   * @brief Callback for ankle compensation from CoM controller
   * @param msg Ankle compensation torques
   */
  void ankleCompensationCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  /**
   * @brief Calculate ZMP position based on CoM and ground reaction forces
   * @return ZMP position in ground plane
   */
  geometry_msgs::msg::Point calculateZMP();

  /**
   * @brief Define the support polygon based on foot contact points
   * For double support, this is the convex hull of both feet
   * For single support, this is the contact area of one foot
   * @return Support polygon vertices
   */
  geometry_msgs::msg::PolygonStamped calculateSupportPolygon();

  /**
   * @brief Check if a point is inside a polygon
   * @param point Point to test
   * @param polygon Polygon vertices
   * @return true if point is inside polygon
   */
  bool isPointInPolygon(
    const geometry_msgs::msg::Point & point,
    const geometry_msgs::msg::Polygon & polygon);

  /**
   * @brief Calculate distance from point to polygon edge (negative if inside)
   * @param point Point to test
   * @param polygon Polygon vertices
   * @return Signed distance (negative inside, positive outside)
   */
  double distanceToPolygonEdge(
    const geometry_msgs::msg::Point & point,
    const geometry_msgs::msg::Polygon & polygon);

  /**
   * @brief Compute ZMP stabilization torques
   * @param zmp_error ZMP error from desired position
   * @return Ankle torque corrections [left, right]
   */
  std::vector<double> computeZMPTorques(const geometry_msgs::msg::Vector3 & zmp_error);

  /**
   * @brief Combine CoM-based ankle torques with ZMP corrections
   * @param com_torques Ankle torques from CoM controller
   * @param zmp_torques ZMP stabilization torques
   * @return Final combined ankle torques
   */
  std::vector<double> combineAnkleTorques(
    const std::vector<double> & com_torques,
    const std::vector<double> & zmp_torques);

  // ROS 2 interfaces
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr com_position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr com_velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr body_tilt_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ankle_comp_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr zmp_position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr support_polygon_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zmp_stable_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr final_ankle_torques_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr zmp_error_pub_;

  // State variables
  geometry_msgs::msg::Point current_com_position_;
  geometry_msgs::msg::Vector3 current_com_velocity_;
  geometry_msgs::msg::Vector3 current_body_tilt_;
  std::vector<double> current_ankle_compensation_;
  geometry_msgs::msg::Point previous_zmp_;
  rclcpp::Time previous_time_;

  bool com_position_received_;
  bool com_velocity_received_;
  bool body_tilt_received_;

  // Parameters
  double foot_length_;
  double foot_width_;
  double zmp_kp_;
  double zmp_kd_;
  double safety_margin_;
  double max_zmp_torque_;

  // Constants
  static constexpr double GRAVITY = 9.81;  // m/s^2
};

}  // namespace balance_control

#endif  // BALANCE_CONTROL__ZMP_STABILIZER_HPP_
