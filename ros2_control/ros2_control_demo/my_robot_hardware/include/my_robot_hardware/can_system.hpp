#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace my_robot_hardware
{

class CanSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CanSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // params
  std::string can_interface_{"can0"};
  int left_can_id_{0x201};
  int right_can_id_{0x202};

  // state
  double left_pos_{0.0}, right_pos_{0.0};
  double left_vel_{0.0}, right_vel_{0.0};

  // command
  double cmd_left_vel_{0.0};
  double cmd_right_vel_{0.0};

  void send_can_velocity(int can_id, double wheel_angular_vel_rad_s);
};

}  // namespace my_robot_hardware
