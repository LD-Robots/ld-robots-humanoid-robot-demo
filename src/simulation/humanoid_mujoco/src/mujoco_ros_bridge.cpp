/**
 * @file mujoco_ros_bridge.cpp
 * @brief ROS 2 bridge for MuJoCo simulation
 *
 * This node provides a hardware interface compatible with ros2_control
 * for MuJoCo simulation.
 */

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace humanoid_mujoco
{

class MuJoCoROSBridge : public rclcpp::Node
{
public:
  MuJoCoROSBridge()
  : Node("mujoco_ros_bridge")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing MuJoCo ROS 2 Bridge");

    // Declare parameters
    this->declare_parameter("control_rate", 100.0);
    this->declare_parameter("num_joints", 0);

    control_rate_ = this->get_parameter("control_rate").as_double();
    num_joints_ = this->get_parameter("num_joints").as_int();

    // Publishers
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu/data", 10);

    left_foot_wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "left_foot/wrench", 10);

    right_foot_wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "right_foot/wrench", 10);

    // Subscribers
    joint_command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "joint_commands", 10,
        std::bind(&MuJoCoROSBridge::jointCommandCallback, this, std::placeholders::_1));

    // Timer for publishing state
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_));
    timer_ = this->create_wall_timer(
        timer_period, std::bind(&MuJoCoROSBridge::publishState, this));

    RCLCPP_INFO(this->get_logger(), "MuJoCo ROS 2 Bridge initialized");
    RCLCPP_INFO(this->get_logger(), "Control rate: %.1f Hz", control_rate_);
  }

private:
  void jointCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != static_cast<size_t>(num_joints_))
    {
      RCLCPP_WARN(this->get_logger(),
          "Received %zu joint commands, expected %d",
          msg->data.size(), num_joints_);
      return;
    }

    joint_commands_ = msg->data;

    // Here you would send commands to MuJoCo
    // This is a simplified version - actual implementation would use
    // shared memory or socket communication with MuJoCo process
  }

  void publishState()
  {
    // Publish joint states
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.header.frame_id = "world";

    // TODO: Get actual joint states from MuJoCo
    // This is a placeholder - actual implementation would read from
    // shared memory or socket communication with MuJoCo process

    joint_state_pub_->publish(joint_state_msg);

    // Publish IMU data
    publishIMU();

    // Publish foot forces
    publishFootForces();
  }

  void publishIMU()
  {
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu_link";

    // TODO: Get actual IMU data from MuJoCo

    imu_pub_->publish(imu_msg);
  }

  void publishFootForces()
  {
    // Left foot
    auto left_wrench_msg = geometry_msgs::msg::WrenchStamped();
    left_wrench_msg.header.stamp = this->now();
    left_wrench_msg.header.frame_id = "left_foot";

    // TODO: Get actual force data from MuJoCo

    left_foot_wrench_pub_->publish(left_wrench_msg);

    // Right foot
    auto right_wrench_msg = geometry_msgs::msg::WrenchStamped();
    right_wrench_msg.header.stamp = this->now();
    right_wrench_msg.header.frame_id = "right_foot";

    // TODO: Get actual force data from MuJoCo

    right_foot_wrench_pub_->publish(right_wrench_msg);
  }

  // Parameters
  double control_rate_;
  int num_joints_;

  // State
  std::vector<double> joint_commands_;

  // ROS 2 interfaces
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr left_foot_wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr right_foot_wrench_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace humanoid_mujoco

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_mujoco::MuJoCoROSBridge>());
  rclcpp::shutdown();
  return 0;
}
