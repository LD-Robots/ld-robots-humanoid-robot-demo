#include "humanoid_hardware/humanoid_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace humanoid_hardware
{

hardware_interface::CallbackReturn HumanoidHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters
  serial_port_ = info_.hardware_parameters["serial_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

  // Initialize vectors
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Initialize IMU data
  hw_imu_orientation_ = {0.0, 0.0, 0.0, 1.0};  // quaternion (x, y, z, w)
  hw_imu_angular_velocity_ = {0.0, 0.0, 0.0};
  hw_imu_linear_acceleration_ = {0.0, 0.0, 9.81};

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Verify joint has required command interfaces
    if (joint.command_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HumanoidHardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Verify joint has required state interfaces
    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HumanoidHardwareInterface"),
        "Joint '%s' has %zu state interfaces. 3 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("HumanoidHardwareInterface"), "Configuring hardware interface...");

  // Initialize all positions to 0
  for (std::size_t i = 0; i < hw_states_positions_.size(); i++)
  {
    hw_states_positions_[i] = 0.0;
    hw_states_velocities_[i] = 0.0;
    hw_states_efforts_[i] = 0.0;
    hw_commands_positions_[i] = 0.0;
    hw_commands_velocities_[i] = 0.0;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("HumanoidHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
HumanoidHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joint state interfaces
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
  }

  // IMU sensor interfaces
  for (const auto & sensor : info_.sensors)
  {
    if (sensor.name == "imu_sensor")
    {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          sensor.name, "orientation.x", &hw_imu_orientation_[0]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          sensor.name, "orientation.y", &hw_imu_orientation_[1]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          sensor.name, "orientation.z", &hw_imu_orientation_[2]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          sensor.name, "orientation.w", &hw_imu_orientation_[3]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          sensor.name, "angular_velocity.x", &hw_imu_angular_velocity_[0]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          sensor.name, "angular_velocity.y", &hw_imu_angular_velocity_[1]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          sensor.name, "angular_velocity.z", &hw_imu_angular_velocity_[2]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          sensor.name, "linear_acceleration.x", &hw_imu_linear_acceleration_[0]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          sensor.name, "linear_acceleration.y", &hw_imu_linear_acceleration_[1]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          sensor.name, "linear_acceleration.z", &hw_imu_linear_acceleration_[2]));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
HumanoidHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn HumanoidHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("HumanoidHardwareInterface"), "Activating hardware interface...");

  // Connect to hardware
  if (!connect_to_hardware())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("HumanoidHardwareInterface"), "Failed to connect to hardware!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("HumanoidHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("HumanoidHardwareInterface"), "Deactivating hardware interface...");

  disconnect_from_hardware();

  RCLCPP_INFO(
    rclcpp::get_logger("HumanoidHardwareInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HumanoidHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read sensor data from hardware
  read_states_from_hardware();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HumanoidHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Send commands to hardware
  send_commands_to_hardware();

  return hardware_interface::return_type::OK;
}

bool HumanoidHardwareInterface::connect_to_hardware()
{
  RCLCPP_INFO(
    rclcpp::get_logger("HumanoidHardwareInterface"),
    "Connecting to hardware on port %s at %d baud", serial_port_.c_str(), baud_rate_);

  // TODO: Implement actual hardware connection
  // This is a placeholder for the actual hardware communication setup
  // You would typically open a serial port, initialize communication, etc.

  return true;
}

bool HumanoidHardwareInterface::disconnect_from_hardware()
{
  RCLCPP_INFO(
    rclcpp::get_logger("HumanoidHardwareInterface"), "Disconnecting from hardware...");

  // TODO: Implement actual hardware disconnection

  return true;
}

bool HumanoidHardwareInterface::send_commands_to_hardware()
{
  // TODO: Implement actual command sending to hardware
  // For now, this is a placeholder that would send the commands
  // stored in hw_commands_positions_ and hw_commands_velocities_
  // to the actual robot hardware via serial/CAN/etc.

  return true;
}

bool HumanoidHardwareInterface::read_states_from_hardware()
{
  // TODO: Implement actual state reading from hardware
  // For now, this is a placeholder that would read the actual
  // joint positions, velocities, efforts, and IMU data from hardware

  // Simple simulation: positions follow commands with some lag
  for (std::size_t i = 0; i < hw_states_positions_.size(); i++)
  {
    if (!std::isnan(hw_commands_positions_[i]))
    {
      double error = hw_commands_positions_[i] - hw_states_positions_[i];
      hw_states_positions_[i] += error * 0.1;  // Simple proportional tracking
      hw_states_velocities_[i] = error * 0.5;
    }
  }

  return true;
}

}  // namespace humanoid_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  humanoid_hardware::HumanoidHardwareInterface, hardware_interface::SystemInterface)
