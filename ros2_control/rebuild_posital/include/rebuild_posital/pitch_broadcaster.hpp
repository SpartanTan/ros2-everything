#pragma once

#include "rebuild_posital/msg/inclination.hpp"
#include "rebuild_posital/pitch_broadcaster_parameters.hpp"

#include "controller_interface/controller_interface.hpp"
#include <memory>
#include <realtime_tools/realtime_publisher.hpp>
#include <string>

namespace rebuild_posital
{
class PitchBroadcaster : public controller_interface::ControllerInterface
{
 public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  std::string sensor_name_{"vehicle_inclinometer"};
  std::string frame_id_{"posital_inclinometer_link"};
  std::string topic_name_{"~/pitch"};
  std::size_t pitch_index_{0};
  std::size_t valid_index_{0};
  std::size_t raw_index_{0};
  std::size_t age_index_{0};
  std::size_t nmt_index_{0};

  std::shared_ptr<pitch_broadcaster::ParamListener> param_listener_{};
  pitch_broadcaster::Params params_;
  
  rclcpp::Publisher<msg::Inclination>::SharedPtr publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<msg::Inclination>> realtime_publisher_;
  msg::Inclination message_;
};
}  // namespace rebuild_posital