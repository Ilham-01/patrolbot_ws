#include "robot_hardware/arduino_diff_drive_system.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

namespace robot_hardware
{

hardware_interface::CallbackReturn ArduinoDiffDriveSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info.joints.size(), 0.0);
  hw_velocities_.resize(info.joints.size(), 0.0);
  hw_commands_.resize(info.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoDiffDriveSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "position", &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "velocity", &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoDiffDriveSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "velocity", &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ArduinoDiffDriveSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoDiffDriveSystem"), "Activating...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoDiffDriveSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoDiffDriveSystem"), "Deactivating...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoDiffDriveSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Placeholder for reading encoder data from Arduino
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoDiffDriveSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Placeholder for sending velocity commands to Arduino
  return hardware_interface::return_type::OK;
}

}  // namespace robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robot_hardware::ArduinoDiffDriveSystem, hardware_interface::SystemInterface)
