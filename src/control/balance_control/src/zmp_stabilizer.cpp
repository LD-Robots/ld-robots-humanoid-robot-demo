#include "balance_control/zmp_stabilizer.hpp"
#include <cmath>
#include <algorithm>

namespace balance_control
{

ZMPStabilizer::ZMPStabilizer(const rclcpp::NodeOptions & options)
: Node("zmp_stabilizer", options),
  com_position_received_(false),
  com_velocity_received_(false),
  body_tilt_received_(false)
{
  // Declare and get parameters
  this->declare_parameter("foot_length", 0.2);
  this->declare_parameter("foot_width", 0.1);
  this->declare_parameter("zmp_kp", 2.0);
  this->declare_parameter("zmp_kd", 0.5);
  this->declare_parameter("safety_margin", 0.02);
  this->declare_parameter("max_zmp_torque", 20.0);

  foot_length_ = this->get_parameter("foot_length").as_double();
  foot_width_ = this->get_parameter("foot_width").as_double();
  zmp_kp_ = this->get_parameter("zmp_kp").as_double();
  zmp_kd_ = this->get_parameter("zmp_kd").as_double();
  safety_margin_ = this->get_parameter("safety_margin").as_double();
  max_zmp_torque_ = this->get_parameter("max_zmp_torque").as_double();

  // Initialize previous ZMP
  previous_zmp_.x = 0.0;
  previous_zmp_.y = 0.0;
  previous_zmp_.z = 0.0;
  previous_time_ = this->now();

  // Create subscribers
  com_position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/balance/com_position",
    10,
    std::bind(&ZMPStabilizer::comPositionCallback, this, std::placeholders::_1));

  com_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/balance/com_velocity",
    10,
    std::bind(&ZMPStabilizer::comVelocityCallback, this, std::placeholders::_1));

  body_tilt_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/balance/body_tilt",
    10,
    std::bind(&ZMPStabilizer::bodyTiltCallback, this, std::placeholders::_1));

  ankle_comp_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/balance/ankle_compensation",
    10,
    std::bind(&ZMPStabilizer::ankleCompensationCallback, this, std::placeholders::_1));

  // Create publishers
  zmp_position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/balance/zmp_position", 10);

  support_polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "/balance/support_polygon", 10);

  zmp_stable_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/balance/zmp_stable", 10);

  final_ankle_torques_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/balance/final_ankle_torques", 10);

  zmp_error_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/balance/zmp_error", 10);

  RCLCPP_INFO(this->get_logger(), "ZMP Stabilizer initialized");
  RCLCPP_INFO(this->get_logger(), "  - Foot dimensions: %.2f m x %.2f m",
              foot_length_, foot_width_);
  RCLCPP_INFO(this->get_logger(), "  - ZMP gains: Kp=%.2f, Kd=%.2f", zmp_kp_, zmp_kd_);
  RCLCPP_INFO(this->get_logger(), "  - Safety margin: %.3f m", safety_margin_);
}

void ZMPStabilizer::comPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  current_com_position_ = msg->point;
  com_position_received_ = true;

  // Only proceed if we have all necessary data
  if (!com_velocity_received_ || !body_tilt_received_) {
    return;
  }

  // Calculate ZMP
  auto zmp = calculateZMP();

  // Calculate support polygon
  auto support_polygon = calculateSupportPolygon();

  // Check if ZMP is stable
  bool is_stable = isPointInPolygon(zmp, support_polygon.polygon);

  // Calculate ZMP error (distance from center of support polygon)
  geometry_msgs::msg::Point polygon_center;
  polygon_center.x = 0.0;
  polygon_center.y = 0.0;
  polygon_center.z = 0.0;

  geometry_msgs::msg::Vector3 zmp_error;
  zmp_error.x = polygon_center.x - zmp.x;
  zmp_error.y = polygon_center.y - zmp.y;
  zmp_error.z = 0.0;

  // Compute ZMP stabilization torques
  auto zmp_torques = computeZMPTorques(zmp_error);

  // Combine with ankle compensation from CoM controller
  auto final_torques = combineAnkleTorques(current_ankle_compensation_, zmp_torques);

  // Publish ZMP position
  geometry_msgs::msg::PointStamped zmp_msg;
  zmp_msg.header.stamp = this->now();
  zmp_msg.header.frame_id = "base_link";
  zmp_msg.point = zmp;
  zmp_position_pub_->publish(zmp_msg);

  // Publish support polygon
  support_polygon_pub_->publish(support_polygon);

  // Publish stability status
  std_msgs::msg::Bool stable_msg;
  stable_msg.data = is_stable;
  zmp_stable_pub_->publish(stable_msg);

  // Publish ZMP error
  geometry_msgs::msg::Vector3Stamped error_msg;
  error_msg.header.stamp = this->now();
  error_msg.header.frame_id = "base_link";
  error_msg.vector = zmp_error;
  zmp_error_pub_->publish(error_msg);

  // Publish final ankle torques
  std_msgs::msg::Float64MultiArray torques_msg;
  torques_msg.data = final_torques;
  final_ankle_torques_pub_->publish(torques_msg);

  // Warn if unstable
  if (!is_stable) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "ZMP outside support polygon! ZMP: [%.3f, %.3f]", zmp.x, zmp.y);
  }

  // Update previous ZMP
  previous_zmp_ = zmp;
  previous_time_ = this->now();
}

void ZMPStabilizer::comVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  current_com_velocity_ = msg->vector;
  com_velocity_received_ = true;
}

void ZMPStabilizer::bodyTiltCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  current_body_tilt_ = msg->vector;
  body_tilt_received_ = true;
}

void ZMPStabilizer::ankleCompensationCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  current_ankle_compensation_ = msg->data;
}

geometry_msgs::msg::Point ZMPStabilizer::calculateZMP()
{
  // ZMP calculation based on simplified dynamics:
  // ZMP = CoM_xy - (CoM_z / g) * CoM_acceleration_xy
  //
  // For static standing (no acceleration), ZMP ≈ CoM projection on ground
  // With body tilt, we need to account for gravitational torque

  geometry_msgs::msg::Point zmp;

  double com_height = current_com_position_.z;

  // Approximate CoM acceleration from body tilt (gravity component)
  double accel_x = GRAVITY * std::sin(current_body_tilt_.y);  // pitch
  double accel_y = GRAVITY * std::sin(current_body_tilt_.x);  // roll

  // ZMP formula
  zmp.x = current_com_position_.x - (com_height / GRAVITY) * accel_x;
  zmp.y = current_com_position_.y - (com_height / GRAVITY) * accel_y;
  zmp.z = 0.0;  // ZMP is always on the ground plane

  return zmp;
}

geometry_msgs::msg::PolygonStamped ZMPStabilizer::calculateSupportPolygon()
{
  // For double support (both feet on ground), create a rectangle
  // representing the convex hull of both feet
  // Assumption: feet are parallel and aligned with Y axis

  geometry_msgs::msg::PolygonStamped polygon;
  polygon.header.stamp = this->now();
  polygon.header.frame_id = "base_link";

  // Assume feet are separated by hip width (approximate)
  double foot_separation = 0.2;  // 20 cm between feet

  // Define four corners of the support polygon (double support)
  geometry_msgs::msg::Point32 p1, p2, p3, p4;

  // Front-left
  p1.x = foot_length_ / 2.0;
  p1.y = foot_separation / 2.0 + foot_width_ / 2.0;
  p1.z = 0.0;

  // Front-right
  p2.x = foot_length_ / 2.0;
  p2.y = -foot_separation / 2.0 - foot_width_ / 2.0;
  p2.z = 0.0;

  // Back-right
  p3.x = -foot_length_ / 2.0;
  p3.y = -foot_separation / 2.0 - foot_width_ / 2.0;
  p3.z = 0.0;

  // Back-left
  p4.x = -foot_length_ / 2.0;
  p4.y = foot_separation / 2.0 + foot_width_ / 2.0;
  p4.z = 0.0;

  polygon.polygon.points.push_back(p1);
  polygon.polygon.points.push_back(p2);
  polygon.polygon.points.push_back(p3);
  polygon.polygon.points.push_back(p4);

  return polygon;
}

bool ZMPStabilizer::isPointInPolygon(
  const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::Polygon & polygon)
{
  // Ray casting algorithm for point-in-polygon test
  int num_vertices = polygon.points.size();
  bool inside = false;

  for (int i = 0, j = num_vertices - 1; i < num_vertices; j = i++) {
    double xi = polygon.points[i].x;
    double yi = polygon.points[i].y;
    double xj = polygon.points[j].x;
    double yj = polygon.points[j].y;

    bool intersect = ((yi > point.y) != (yj > point.y)) &&
                     (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi);

    if (intersect) {
      inside = !inside;
    }
  }

  return inside;
}

double ZMPStabilizer::distanceToPolygonEdge(
  const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::Polygon & polygon)
{
  // Calculate minimum distance to polygon edges
  double min_distance = std::numeric_limits<double>::max();
  int num_vertices = polygon.points.size();

  for (int i = 0; i < num_vertices; ++i) {
    int j = (i + 1) % num_vertices;

    double x1 = polygon.points[i].x;
    double y1 = polygon.points[i].y;
    double x2 = polygon.points[j].x;
    double y2 = polygon.points[j].y;

    // Distance from point to line segment
    double dx = x2 - x1;
    double dy = y2 - y1;
    double length_sq = dx * dx + dy * dy;

    if (length_sq == 0.0) {
      // Degenerate edge
      double dist = std::hypot(point.x - x1, point.y - y1);
      min_distance = std::min(min_distance, dist);
      continue;
    }

    double t = std::clamp(((point.x - x1) * dx + (point.y - y1) * dy) / length_sq, 0.0, 1.0);
    double proj_x = x1 + t * dx;
    double proj_y = y1 + t * dy;

    double dist = std::hypot(point.x - proj_x, point.y - proj_y);
    min_distance = std::min(min_distance, dist);
  }

  // Return negative if inside polygon
  bool inside = isPointInPolygon(point, polygon);
  return inside ? -min_distance : min_distance;
}

std::vector<double> ZMPStabilizer::computeZMPTorques(const geometry_msgs::msg::Vector3 & zmp_error)
{
  // PD controller for ZMP stabilization
  std::vector<double> torques(2, 0.0);  // [left_ankle, right_ankle]

  // Calculate ZMP velocity (finite differences)
  double dt = (this->now() - previous_time_).seconds();
  geometry_msgs::msg::Vector3 zmp_velocity;

  if (dt > 0.0 && dt < 1.0) {
    zmp_velocity.x = (previous_zmp_.x - current_com_position_.x) / dt;
    zmp_velocity.y = (previous_zmp_.y - current_com_position_.y) / dt;
  } else {
    zmp_velocity.x = 0.0;
    zmp_velocity.y = 0.0;
  }

  // Pitch (sagittal plane) correction - both ankles
  double pitch_torque = zmp_kp_ * zmp_error.x + zmp_kd_ * zmp_velocity.x;
  torques[0] += pitch_torque;
  torques[1] += pitch_torque;

  // Roll (frontal plane) correction - differential ankle torque
  double roll_torque = zmp_kp_ * zmp_error.y + zmp_kd_ * zmp_velocity.y;
  torques[0] += roll_torque;   // Left ankle
  torques[1] -= roll_torque;   // Right ankle (opposite)

  // Clamp to max torque
  for (auto & torque : torques) {
    torque = std::clamp(torque, -max_zmp_torque_, max_zmp_torque_);
  }

  return torques;
}

std::vector<double> ZMPStabilizer::combineAnkleTorques(
  const std::vector<double> & com_torques,
  const std::vector<double> & zmp_torques)
{
  // Combine CoM and ZMP torques with weighting
  // ZMP has higher priority for immediate stability
  double com_weight = 0.4;
  double zmp_weight = 0.6;

  std::vector<double> combined(2, 0.0);

  if (com_torques.size() >= 2 && zmp_torques.size() >= 2) {
    combined[0] = com_weight * com_torques[0] + zmp_weight * zmp_torques[0];
    combined[1] = com_weight * com_torques[1] + zmp_weight * zmp_torques[1];
  } else if (zmp_torques.size() >= 2) {
    combined = zmp_torques;
  }

  // Final clamping
  for (auto & torque : combined) {
    torque = std::clamp(torque, -max_zmp_torque_, max_zmp_torque_);
  }

  return combined;
}

}  // namespace balance_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(balance_control::ZMPStabilizer)
