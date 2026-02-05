#include "my_robot_hardware/can_system.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <cstdlib>
#include <cmath>

namespace my_robot_hardware
{

hardware_interface::CallbackReturn CanSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters["can_interface"];
  }
  if (info_.hardware_parameters.count("left_can_id")) {
    left_can_id_ = std::strtol(info_.hardware_parameters["left_can_id"].c_str(), nullptr, 0);
  }
  if (info_.hardware_parameters.count("right_can_id")) {
    right_can_id_ = std::strtol(info_.hardware_parameters["right_can_id"].c_str(), nullptr, 0);
  }

  left_pos_ = right_pos_ = 0.0;
  left_vel_ = right_vel_ = 0.0;
  cmd_left_vel_ = cmd_right_vel_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanSystem::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("my_robot_hardware::CanSystem"),
              "Configured: can_interface=%s left_id=0x%X right_id=0x%X",
              can_interface_.c_str(), left_can_id_, right_can_id_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("my_robot_hardware::CanSystem"), "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("my_robot_hardware::CanSystem"), "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CanSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  states.emplace_back("left_wheel_joint", hardware_interface::HW_IF_POSITION, &left_pos_);
  states.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &left_vel_);
  states.emplace_back("right_wheel_joint", hardware_interface::HW_IF_POSITION, &right_pos_);
  states.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &right_vel_);
  return states;
}

std::vector<hardware_interface::CommandInterface> CanSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmds;
  cmds.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &cmd_left_vel_);
  cmds.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &cmd_right_vel_);
  return cmds;
}

hardware_interface::return_type CanSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  // 示例：用命令速度模拟状态速度，并积分位置
  const double dt = period.seconds();
  left_vel_ = cmd_left_vel_;
  right_vel_ = cmd_right_vel_;
  left_pos_ += left_vel_ * dt;
  right_pos_ += right_vel_ * dt;
  return hardware_interface::return_type::OK;
}

void CanSystem::send_can_velocity(int can_id, double wheel_angular_vel_rad_s)
{
  const int32_t scaled = static_cast<int32_t>(std::lround(wheel_angular_vel_rad_s * 1000.0));
  RCLCPP_INFO(rclcpp::get_logger("my_robot_hardware::CanSystem"),
              "[CAN:%s] ID=0x%X vel=%.3f rad/s payload(int32)= %d",
              can_interface_.c_str(), can_id, wheel_angular_vel_rad_s, scaled);
}

hardware_interface::return_type CanSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  send_can_velocity(left_can_id_, cmd_left_vel_);
  send_can_velocity(right_can_id_, cmd_right_vel_);
  return hardware_interface::return_type::OK;
}

}  // namespace my_robot_hardware

PLUGINLIB_EXPORT_CLASS(my_robot_hardware::CanSystem, hardware_interface::SystemInterface)
