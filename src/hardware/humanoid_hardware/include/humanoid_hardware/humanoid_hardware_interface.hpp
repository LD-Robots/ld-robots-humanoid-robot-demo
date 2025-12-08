#ifndef HUMANOID_HARDWARE__HUMANOID_HARDWARE_INTERFACE_HPP_
#define HUMANOID_HARDWARE__HUMANOID_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace humanoid_hardware
{

class HumanoidHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HumanoidHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters
  std::string serial_port_;
  int baud_rate_;

  // Store the command and state values
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_efforts_;

  // IMU data
  std::array<double, 4> hw_imu_orientation_;
  std::array<double, 3> hw_imu_angular_velocity_;
  std::array<double, 3> hw_imu_linear_acceleration_;

  // Communication
  bool connect_to_hardware();
  bool disconnect_from_hardware();
  bool send_commands_to_hardware();
  bool read_states_from_hardware();
};

}  // namespace humanoid_hardware

#endif  // HUMANOID_HARDWARE__HUMANOID_HARDWARE_INTERFACE_HPP_
