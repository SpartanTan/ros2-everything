#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <string>
#include <vector>

namespace my_robot_controllers
{

class CmdVelDiffDriveController : public controller_interface::ControllerInterface
{
public:
  CmdVelDiffDriveController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // params
  std::string left_wheel_joint_{"left_wheel_joint"};
  std::string right_wheel_joint_{"right_wheel_joint"};
  double wheel_radius_{0.1};
  double wheel_separation_{0.4};

  // latest cmd_vel
  double last_v_{0.0};
  double last_w_{0.0};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  double cmd_timeout_{0.5};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  // command interface indices
  int left_cmd_index_{-1};
  int right_cmd_index_{-1};

  void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  static int find_command_index(
    const std::vector<hardware_interface::LoanedCommandInterface> & ifaces,
    const std::string & joint_name,
    const std::string & interface_name);
};

}  // namespace my_robot_controllers
