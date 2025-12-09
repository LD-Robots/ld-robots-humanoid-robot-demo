#ifndef BALANCE_CONTROL__IMU_FEEDBACK_PROCESSOR_HPP_
#define BALANCE_CONTROL__IMU_FEEDBACK_PROCESSOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>

namespace balance_control
{

/**
 * @brief IMU Feedback Processor Node
 *
 * This node processes raw IMU data and provides filtered orientation and angular velocity
 * information for balance control. It applies complementary filtering to combine
 * accelerometer and gyroscope data for robust orientation estimation.
 *
 * Subscribed Topics:
 *   - /imu/data (sensor_msgs/msg/Imu): Raw IMU measurements
 *
 * Published Topics:
 *   - /balance/body_orientation (geometry_msgs/msg/QuaternionStamped): Filtered body orientation
 *   - /balance/angular_velocity (geometry_msgs/msg/Vector3Stamped): Filtered angular velocity
 *   - /balance/body_tilt (geometry_msgs/msg/Vector3Stamped): Roll, pitch, yaw tilt angles
 *
 * Parameters:
 *   - complementary_filter_alpha (double): Filter coefficient (0.0-1.0), default: 0.98
 *   - gyro_bias_threshold (double): Gyroscope bias detection threshold (rad/s), default: 0.01
 *   - max_tilt_angle (double): Maximum allowable tilt before emergency (rad), default: 0.5
 */
class IMUFeedbackProcessor : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options for ROS 2 configuration
   */
  explicit IMUFeedbackProcessor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~IMUFeedbackProcessor() = default;

private:
  /**
   * @brief Callback for incoming IMU messages
   * @param msg IMU message containing orientation, angular velocity, and linear acceleration
   */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief Apply complementary filter to fuse accelerometer and gyroscope data
   * @param accel_orientation Orientation from accelerometer
   * @param gyro_rate Angular velocity from gyroscope
   * @param dt Time step
   */
  void applyComplementaryFilter(
    const geometry_msgs::msg::Quaternion & accel_orientation,
    const geometry_msgs::msg::Vector3 & gyro_rate,
    double dt);

  /**
   * @brief Convert quaternion to roll-pitch-yaw Euler angles
   * @param quat Input quaternion
   * @return Vector3 with roll, pitch, yaw angles (radians)
   */
  geometry_msgs::msg::Vector3 quaternionToRPY(const geometry_msgs::msg::Quaternion & quat);

  /**
   * @brief Check if the robot is tilting beyond safe limits
   * @param rpy Roll, pitch, yaw angles
   * @return true if tilt exceeds maximum threshold
   */
  bool isTiltExcessive(const geometry_msgs::msg::Vector3 & rpy);

  // ROS 2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr orientation_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr angular_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr body_tilt_pub_;

  // TF2 for coordinate transformations
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Filter state
  geometry_msgs::msg::Quaternion filtered_orientation_;
  geometry_msgs::msg::Vector3 filtered_angular_velocity_;
  rclcpp::Time last_imu_time_;
  bool first_imu_received_;

  // Parameters
  double complementary_filter_alpha_;
  double gyro_bias_threshold_;
  double max_tilt_angle_;

  // Gyroscope bias estimation
  geometry_msgs::msg::Vector3 gyro_bias_;
  int bias_sample_count_;
  static constexpr int BIAS_SAMPLE_SIZE = 100;
};

}  // namespace balance_control

#endif  // BALANCE_CONTROL__IMU_FEEDBACK_PROCESSOR_HPP_
