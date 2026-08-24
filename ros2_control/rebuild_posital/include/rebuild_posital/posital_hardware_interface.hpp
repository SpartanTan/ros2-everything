#pragma once

#include <canopen_core/device_container.hpp>
#include <canopen_core/exchange.hpp>
#include <canopen_proxy_driver/proxy_driver.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rebuild_posital
{
class PositalHardwareInterface : public hardware_interface::SensorInterface
{
 public:
  PositalHardwareInterface() = default;
  ~PositalHardwareInterface() override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
      override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

 private:
  struct Sample
  {
    std::uint16_t raw{0};
    std::uint8_t nmt_state{0};
    std::uint16_t emcy_error_code{0};
    std::chrono::steady_clock::time_point received_at{};
    bool received{false};
  };

  // CAN callbacks may run concurrently, so they serialize updates to the
  // non-RT snapshot. read() only accesses sample_buffer_ and never waits for
  // this mutex.
  std::mutex sample_write_mutex_;
  Sample non_rt_sample_;
  realtime_tools::RealtimeBuffer<Sample> sample_buffer_{Sample{}};
  void initialize_canopen();
  void stop_canopen();
  void register_driver_callbacks();
  void on_rpdo(const ros2_canopen::COData data, std::uint8_t node_id);
  void on_nmt(canopen::NmtState state, std::uint8_t node_id);
  void on_emcy(ros2_canopen::COEmcy emcy, std::uint8_t node_id);

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<ros2_canopen::DeviceContainer> device_container_;
  std::thread spin_thread_;
  std::thread init_thread_;
  std::exception_ptr init_exception_;
  std::atomic<bool> configured_{false};
  std::atomic<bool> active_{false};

  std::string bus_config_;
  std::string master_config_;
  std::string master_bin_;
  std::string can_interface_{"pcan1"};
  std::string sensor_name_{"vehicle_inclinometer"};
  std::uint8_t node_id_{1};
  int sample_timeout_ms_{250};
  double counts_per_degree_{100.0};
  double direction_{1.0};
  double mounting_offset_rad_{0.0};

  double pitch_rad_{0.0};
  double valid_{0.0};
  double raw_value_{0.0};
  double sample_age_sec_{0.0};
  double nmt_state_{0.0};

  rclcpp::Logger logger_{rclcpp::get_logger("PositalHardwareInterface")};
};
}  // namespace rebuild_posital
