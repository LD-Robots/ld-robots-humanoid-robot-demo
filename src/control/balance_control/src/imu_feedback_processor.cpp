#include "balance_control/imu_feedback_processor.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace balance_control
{

IMUFeedbackProcessor::IMUFeedbackProcessor(const rclcpp::NodeOptions & options)
: Node("imu_feedback_processor", options),
  first_imu_received_(false),
  bias_sample_count_(0)
{
  // Declare and get parameters
  this->declare_parameter("complementary_filter_alpha", 0.98);
  this->declare_parameter("gyro_bias_threshold", 0.01);
  this->declare_parameter("max_tilt_angle", 0.5);  // ~28.6 degrees

  complementary_filter_alpha_ = this->get_parameter("complementary_filter_alpha").as_double();
  gyro_bias_threshold_ = this->get_parameter("gyro_bias_threshold").as_double();
  max_tilt_angle_ = this->get_parameter("max_tilt_angle").as_double();

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize filtered orientation to identity quaternion
  filtered_orientation_.w = 1.0;
  filtered_orientation_.x = 0.0;
  filtered_orientation_.y = 0.0;
  filtered_orientation_.z = 0.0;

  // Initialize gyro bias to zero
  gyro_bias_.x = 0.0;
  gyro_bias_.y = 0.0;
  gyro_bias_.z = 0.0;

  // Create subscriber
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data",
    rclcpp::SensorDataQoS(),
    std::bind(&IMUFeedbackProcessor::imuCallback, this, std::placeholders::_1));

  // Create publishers
  orientation_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/balance/body_orientation", 10);

  angular_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/balance/angular_velocity", 10);

  body_tilt_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/balance/body_tilt", 10);

  RCLCPP_INFO(this->get_logger(), "IMU Feedback Processor initialized");
  RCLCPP_INFO(this->get_logger(), "  - Complementary filter alpha: %.2f", complementary_filter_alpha_);
  RCLCPP_INFO(this->get_logger(), "  - Max tilt angle: %.2f rad (%.1f deg)",
              max_tilt_angle_, max_tilt_angle_ * 180.0 / M_PI);
}

void IMUFeedbackProcessor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Estimate gyroscope bias during initial samples
  if (bias_sample_count_ < BIAS_SAMPLE_SIZE) {
    gyro_bias_.x += msg->angular_velocity.x;
    gyro_bias_.y += msg->angular_velocity.y;
    gyro_bias_.z += msg->angular_velocity.z;
    bias_sample_count_++;

    if (bias_sample_count_ == BIAS_SAMPLE_SIZE) {
      gyro_bias_.x /= BIAS_SAMPLE_SIZE;
      gyro_bias_.y /= BIAS_SAMPLE_SIZE;
      gyro_bias_.z /= BIAS_SAMPLE_SIZE;
      RCLCPP_INFO(this->get_logger(), "Gyro bias calibrated: [%.4f, %.4f, %.4f] rad/s",
                  gyro_bias_.x, gyro_bias_.y, gyro_bias_.z);
    }
    return;
  }

  // Initialize time on first measurement
  if (!first_imu_received_) {
    last_imu_time_ = msg->header.stamp;
    filtered_orientation_ = msg->orientation;
    first_imu_received_ = true;
    return;
  }

  // Calculate time step
  rclcpp::Time current_time(msg->header.stamp);
  double dt = (current_time - last_imu_time_).seconds();
  last_imu_time_ = current_time;

  // Remove bias from gyroscope measurements
  geometry_msgs::msg::Vector3 corrected_angular_velocity;
  corrected_angular_velocity.x = msg->angular_velocity.x - gyro_bias_.x;
  corrected_angular_velocity.y = msg->angular_velocity.y - gyro_bias_.y;
  corrected_angular_velocity.z = msg->angular_velocity.z - gyro_bias_.z;

  // Apply complementary filter
  applyComplementaryFilter(msg->orientation, corrected_angular_velocity, dt);

  // Store filtered angular velocity
  filtered_angular_velocity_ = corrected_angular_velocity;

  // Convert to roll-pitch-yaw
  auto rpy = quaternionToRPY(filtered_orientation_);

  // Check for excessive tilt
  if (isTiltExcessive(rpy)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Excessive body tilt detected! Roll: %.2f, Pitch: %.2f (deg)",
                         rpy.x * 180.0 / M_PI, rpy.y * 180.0 / M_PI);
  }

  // Publish filtered orientation
  geometry_msgs::msg::PoseStamped orientation_msg;
  orientation_msg.header = msg->header;
  orientation_msg.pose.orientation = filtered_orientation_;
  orientation_pub_->publish(orientation_msg);

  // Publish angular velocity
  geometry_msgs::msg::Vector3Stamped angular_velocity_msg;
  angular_velocity_msg.header = msg->header;
  angular_velocity_msg.vector = filtered_angular_velocity_;
  angular_velocity_pub_->publish(angular_velocity_msg);

  // Publish body tilt (RPY)
  geometry_msgs::msg::Vector3Stamped tilt_msg;
  tilt_msg.header = msg->header;
  tilt_msg.vector = rpy;
  body_tilt_pub_->publish(tilt_msg);
}

void IMUFeedbackProcessor::applyComplementaryFilter(
  const geometry_msgs::msg::Quaternion & accel_orientation,
  const geometry_msgs::msg::Vector3 & gyro_rate,
  double dt)
{
  // Complementary filter:
  // filtered = alpha * (gyro integration) + (1 - alpha) * (accelerometer)

  // Gyro integration (simplified - for production use proper quaternion integration)
  tf2::Quaternion q_gyro;
  q_gyro.setRPY(gyro_rate.x * dt, gyro_rate.y * dt, gyro_rate.z * dt);

  tf2::Quaternion q_filtered;
  tf2::fromMsg(filtered_orientation_, q_filtered);
  q_filtered = q_filtered * q_gyro;
  q_filtered.normalize();

  // Accelerometer orientation
  tf2::Quaternion q_accel;
  tf2::fromMsg(accel_orientation, q_accel);

  // Apply complementary filter
  q_filtered = q_filtered.slerp(q_accel, 1.0 - complementary_filter_alpha_);
  q_filtered.normalize();

  filtered_orientation_ = tf2::toMsg(q_filtered);
}

geometry_msgs::msg::Vector3 IMUFeedbackProcessor::quaternionToRPY(
  const geometry_msgs::msg::Quaternion & quat)
{
  tf2::Quaternion tf_quat;
  tf2::fromMsg(quat, tf_quat);

  tf2::Matrix3x3 m(tf_quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  geometry_msgs::msg::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;

  return rpy;
}

bool IMUFeedbackProcessor::isTiltExcessive(const geometry_msgs::msg::Vector3 & rpy)
{
  return (std::abs(rpy.x) > max_tilt_angle_ || std::abs(rpy.y) > max_tilt_angle_);
}

}  // namespace balance_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(balance_control::IMUFeedbackProcessor)
