#include "rebuild_posital/pitch_broadcaster.hpp"

#include "controller_interface/controller_interface.hpp"
#include <cmath>
#include <cstdint>
#include <limits>

namespace rebuild_posital
{
controller_interface::CallbackReturn PitchBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<pitch_broadcaster::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception& e)
  {
    return CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PitchBroadcaster::command_interface_configuration()
    const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration PitchBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names = {sensor_name_ + "/pitch", sensor_name_ + "/valid", sensor_name_ + "/raw",
                         sensor_name_ + "/sample_age", sensor_name_ + "/nmt_state"};
  return configuration;
}

controller_interface::CallbackReturn PitchBroadcaster::on_configure(const rclcpp_lifecycle::State&)
{
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
  sensor_name_ = params_.sensor_name;
  frame_id_ = params_.frame_id;
  topic_name_ = params_.topic_name;
  publisher_ = get_node()->create_publisher<msg::Inclination>(topic_name_, rclcpp::SensorDataQoS());
  realtime_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<msg::Inclination>>(publisher_);
  message_.header.frame_id = frame_id_;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PitchBroadcaster::on_activate(const rclcpp_lifecycle::State&)
{
  auto index_of = [this](const std::string& full_name)
  {
    for (std::size_t index = 0; index < state_interfaces_.size(); ++index)
    {
      const auto candidate = state_interfaces_[index].get_prefix_name() + "/" +
                             state_interfaces_[index].get_interface_name();
      if (candidate == full_name)
      {
        return index;
      }
    }
    return std::numeric_limits<std::size_t>::max();
  };

  pitch_index_ = index_of(sensor_name_ + "/pitch");
  valid_index_ = index_of(sensor_name_ + "/valid");
  raw_index_ = index_of(sensor_name_ + "/raw");
  age_index_ = index_of(sensor_name_ + "/sample_age");
  nmt_index_ = index_of(sensor_name_ + "/nmt_state");

  if (pitch_index_ == std::numeric_limits<std::size_t>::max() ||
      valid_index_ == std::numeric_limits<std::size_t>::max() ||
      raw_index_ == std::numeric_limits<std::size_t>::max() ||
      age_index_ == std::numeric_limits<std::size_t>::max() ||
      nmt_index_ == std::numeric_limits<std::size_t>::max())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "One or more POSITAL state interfaces are missing");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PitchBroadcaster::on_deactivate(const rclcpp_lifecycle::State&)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PitchBroadcaster::update(const rclcpp::Time& time,
                                                           const rclcpp::Duration&)
{
  if (!realtime_publisher_)
  {
    return controller_interface::return_type::OK;
  }

  auto& message = message_;
  message.header.stamp = time;
  message.header.frame_id = frame_id_;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  message.pitch_rad = state_interfaces_[pitch_index_].get_optional().value_or(nan);
  message.valid = state_interfaces_[valid_index_].get_optional().value_or(0.0) > 0.5;
  const double raw = state_interfaces_[raw_index_].get_optional().value_or(nan);
  message.raw_value = std::isfinite(raw) ? static_cast<std::uint16_t>(raw) : 0;
  message.sample_age_sec = state_interfaces_[age_index_].get_optional().value_or(nan);
  const double nmt = state_interfaces_[nmt_index_].get_optional().value_or(nan);
  message.nmt_state = std::isfinite(nmt) ? static_cast<std::uint8_t>(nmt) : 0;
  realtime_publisher_->try_publish(message);
  return controller_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rebuild_posital::PitchBroadcaster, controller_interface::ControllerInterface)

}  // namespace rebuild_posital