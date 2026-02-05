#include "my_robot_controllers/cmdvel_diffdrive_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <algorithm>
#include <cmath>

namespace my_robot_controllers
{

controller_interface::CallbackReturn CmdVelDiffDriveController::on_init()
{
  try {
    auto_declare<std::string>("left_wheel_joint", left_wheel_joint_);
    auto_declare<std::string>("right_wheel_joint", right_wheel_joint_);
    auto_declare<double>("wheel_radius", wheel_radius_);
    auto_declare<double>("wheel_separation", wheel_separation_);
    auto_declare<double>("cmd_timeout", cmd_timeout_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CmdVelDiffDriveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = {
    left_wheel_joint_ + "/" + hardware_interface::HW_IF_VELOCITY,
    right_wheel_joint_ + "/" + hardware_interface::HW_IF_VELOCITY
  };
  return cfg;
}

controller_interface::InterfaceConfiguration
CmdVelDiffDriveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::NONE;
  return cfg;
}

controller_interface::CallbackReturn
CmdVelDiffDriveController::on_configure(const rclcpp_lifecycle::State &)
{
  left_wheel_joint_ = get_node()->get_parameter("left_wheel_joint").as_string();
  right_wheel_joint_ = get_node()->get_parameter("right_wheel_joint").as_string();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  wheel_separation_ = get_node()->get_parameter("wheel_separation").as_double();
  cmd_timeout_ = get_node()->get_parameter("cmd_timeout").as_double();

  cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10),
    std::bind(&CmdVelDiffDriveController::cmd_cb, this, std::placeholders::_1));

  last_v_ = 0.0;
  last_w_ = 0.0;
  last_cmd_time_ = get_node()->now();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CmdVelDiffDriveController::on_activate(const rclcpp_lifecycle::State &)
{
  left_cmd_index_ = find_command_index(command_interfaces_, left_wheel_joint_, hardware_interface::HW_IF_VELOCITY);
  right_cmd_index_ = find_command_index(command_interfaces_, right_wheel_joint_, hardware_interface::HW_IF_VELOCITY);

  if (left_cmd_index_ < 0 || right_cmd_index_ < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to find command interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_interfaces_[left_cmd_index_].set_value(0.0);
  command_interfaces_[right_cmd_index_].set_value(0.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CmdVelDiffDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (left_cmd_index_ >= 0) command_interfaces_[left_cmd_index_].set_value(0.0);
  if (right_cmd_index_ >= 0) command_interfaces_[right_cmd_index_].set_value(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

void CmdVelDiffDriveController::cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_v_ = msg->linear.x;
  last_w_ = msg->angular.z;
  last_cmd_time_ = get_node()->now();
}

int CmdVelDiffDriveController::find_command_index(
  const std::vector<hardware_interface::LoanedCommandInterface> & ifaces,
  const std::string & joint_name,
  const std::string & interface_name)
{
  const std::string full = joint_name + "/" + interface_name;
  for (size_t i = 0; i < ifaces.size(); ++i) {
    if (ifaces[i].get_full_name() == full) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

controller_interface::return_type
CmdVelDiffDriveController::update(const rclcpp::Time & time, const rclcpp::Duration &)
{
  // timeout -> stop
  if ((time - last_cmd_time_).seconds() > cmd_timeout_) {
    last_v_ = 0.0;
    last_w_ = 0.0;
  }

  // diff drive inverse kinematics:
  // v_l = v - w*L/2, v_r = v + w*L/2
  // omega_l = v_l / R, omega_r = v_r / R
  const double v_l = last_v_ - last_w_ * (wheel_separation_ * 0.5);
  const double v_r = last_v_ + last_w_ * (wheel_separation_ * 0.5);
  const double omega_l = v_l / wheel_radius_;
  const double omega_r = v_r / wheel_radius_;

  command_interfaces_[left_cmd_index_].set_value(omega_l);
  command_interfaces_[right_cmd_index_].set_value(omega_r);

  return controller_interface::return_type::OK;
}

}  // namespace my_robot_controllers

PLUGINLIB_EXPORT_CLASS(my_robot_controllers::CmdVelDiffDriveController,
                       controller_interface::ControllerInterface)
