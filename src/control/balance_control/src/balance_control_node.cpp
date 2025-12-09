/**
 * @file balance_control_node.cpp
 * @brief Main executable for balance control system
 *
 * This standalone executable can run the balance control nodes
 * individually or as components. Use the launch file for
 * proper multi-node execution.
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create a multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;

  // Note: Individual nodes should be launched via the launch file
  // This executable is primarily for component-based launching

  RCLCPP_INFO(rclcpp::get_logger("balance_control_node"),
              "Balance Control Node - Use launch file to start system");

  rclcpp::spin(std::make_shared<rclcpp::Node>("balance_control_node"));

  rclcpp::shutdown();
  return 0;
}
