#include "rebuild_posital/posital_hardware_interface.hpp"

#include "rebuild_posital/protocol.hpp"

#include <canopen_proxy_driver/proxy_driver.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rebuild_posital
{

constexpr std::uint8_t kOperationalNmtState = 0x05;

PositalHardwareInterface::~PositalHardwareInterface() { stop_canopen(); }

hardware_interface::CallbackReturn PositalHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  try
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    device_container_ = std::make_shared<ros2_canopen::DeviceContainer>(executor_);
    executor_->add_node(device_container_);

    spin_thread_ = std::thread(
        [this]()
        {
          executor_
              ->spin();  // keep running and take care of all callbacks until on_cleanup/on_shutdown
          executor_->remove_node(device_container_);
        });
    init_exception_ = nullptr;
    init_thread_ = std::thread(&PositalHardwareInterface::initialize_canopen, this);
    init_thread_.join();
    if (init_exception_)
    {
      std::rethrow_exception(init_exception_);
    }
    register_driver_callbacks();
    configured_.store(true);
  }
  catch (const std::exception& exception)
  {
    RCLCPP_ERROR(logger_, "ros2_canopen initialization failed: %s", exception.what());
    stop_canopen();
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "Configured POSITAL through ros2_canopen on %s (node %u)",
              can_interface_.c_str(), node_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn PositalHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State& previous_state)
{
  stop_canopen();
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn PositalHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State& previous_state)
{
  stop_canopen();
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn PositalHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
  if (!configured_.load())
  {
    RCLCPP_ERROR(logger_, "Cannot activate before ros2_canopen is configured");
    return hardware_interface::CallbackReturn::ERROR;
  }
  {
    std::lock_guard<std::mutex> lock(sample_write_mutex_);
    non_rt_sample_.received = false;
    sample_buffer_.writeFromNonRT(non_rt_sample_);
  }
  active_.store(true);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PositalHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& previous_state)
{
  active_.store(false);
  valid_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PositalHardwareInterface::read(const rclcpp::Time& time,
                                                               const rclcpp::Duration& period)
{
  const auto now = std::chrono::steady_clock::now();
  const Sample sample = *sample_buffer_.readFromRT();
  raw_value_ = static_cast<double>(sample.raw);
  nmt_state_ = static_cast<double>(sample.nmt_state);

  if (!sample.received)
  {
    pitch_rad_ = std::numeric_limits<double>::quiet_NaN();
    sample_age_sec_ = std::numeric_limits<double>::infinity();
    valid_ = 0.0;
    return hardware_interface::return_type::OK;
  }

  sample_age_sec_ = std::chrono::duration<double>(now - sample.received_at).count();
  pitch_rad_ = decode_pitch_rad(sample.raw, counts_per_degree_, direction_, mounting_offset_rad_);
  valid_ = active_.load() && sample.nmt_state == kOperationalNmtState &&
                   sample.emcy_error_code == 0 &&
                   sample_age_sec_ <= static_cast<double>(sample_timeout_ms_) / 1000.0
               ? 1.0
               : 0.0;
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn PositalHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SensorInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.sensors.size() != 1)
  {
    RCLCPP_ERROR(logger_, "Expected exactly one <sensor>, got %zu", info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  sensor_name_ = info_.sensors.front().name;

  const auto& parameters = info_.hardware_parameters;
  auto required_parameter = [&parameters](const std::string& name) -> std::string
  {
    const auto it = parameters.find(name);
    if (it == parameters.end() || it->second.empty())
    {
      throw std::invalid_argument("Missing hardware parameter '" + name + "'");
    }
    return it->second;
  };
  auto parameter = [&parameters](const std::string& name, const std::string& fallback)
  {
    const auto it = parameters.find(name);
    return it == parameters.end() ? fallback : it->second;
  };

  try
  {
    bus_config_ = required_parameter("bus_config");
    master_config_ = required_parameter("master_config");
    master_bin_ = parameter("master_bin", "");
    if (master_bin_ == "\"\"")
    {
      master_bin_.clear();
    }
    can_interface_ = parameter("can_interface_name", can_interface_);
    const auto parsed_node_id = std::stoul(parameter("node_id", "1"), nullptr, 0);
    if (parsed_node_id == 0 || parsed_node_id > 127)
    {
      throw std::invalid_argument("node_id must be in the range [1, 127]");
    }
    node_id_ = static_cast<std::uint8_t>(parsed_node_id);
    sample_timeout_ms_ = std::stoi(parameter("sample_timeout_ms", "250"));
    counts_per_degree_ = std::stod(parameter("counts_per_degree", "100.0"));
    direction_ = std::stod(parameter("direction", "1.0"));
    mounting_offset_rad_ = std::stod(parameter("mounting_offset_rad", "0.0"));
  }
  catch (const std::exception& exception)
  {
    RCLCPP_ERROR(logger_, "Invalid hardware parameter: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (sample_timeout_ms_ <= 0 || counts_per_degree_ <= 0.0 ||
      !std::isfinite(direction_) || direction_ == 0.0 || !std::isfinite(mounting_offset_rad_))
  {
    RCLCPP_ERROR(logger_, "Invalid node_id, timeout, scale, direction, or offset parameter");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto& state_interfaces = info_.sensors.front().state_interfaces;
  const std::vector<std::string> expected{"pitch", "valid", "raw", "sample_age", "nmt_state"};
  for (const auto& name : expected)
  {
    bool found = false;
    for (const auto& interface : state_interfaces)
    {
      found = found || interface.name == name;
    }
    if (!found)
    {
      RCLCPP_ERROR(logger_, "Missing state interface '%s/%s'", sensor_name_.c_str(), name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  pitch_rad_ = std::numeric_limits<double>::quiet_NaN();
  valid_ = 0.0;
  raw_value_ = 0.0;
  sample_age_sec_ = std::numeric_limits<double>::infinity();
  nmt_state_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface::ConstSharedPtr>
PositalHardwareInterface::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> interfaces;
  interfaces.emplace_back(
      std::make_shared<hardware_interface::StateInterface>(sensor_name_, "pitch", &pitch_rad_));
  interfaces.emplace_back(
      std::make_shared<hardware_interface::StateInterface>(sensor_name_, "valid", &valid_));
  interfaces.emplace_back(
      std::make_shared<hardware_interface::StateInterface>(sensor_name_, "raw", &raw_value_));
  interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "sample_age", &sample_age_sec_));
  interfaces.emplace_back(
      std::make_shared<hardware_interface::StateInterface>(sensor_name_, "nmt_state", &nmt_state_));
  return interfaces;
}

void PositalHardwareInterface::initialize_canopen()
{
  try
  {
    device_container_->init(can_interface_, master_config_, bus_config_, master_bin_);
  }
  catch (...)
  {
    init_exception_ = std::current_exception();
  }
}

void PositalHardwareInterface::stop_canopen()
{
  active_.store(false);
  configured_.store(false);
  if (executor_)
  {
    executor_->cancel();
  }
  if (init_thread_.joinable())
  {
    init_thread_.join();
  }
  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }
  device_container_.reset();
  executor_.reset();
}

void PositalHardwareInterface::register_driver_callbacks()
{
  const auto& drivers = device_container_->get_registered_drivers();
  const auto driver_it = drivers.find(node_id_);
  if (driver_it == drivers.end())
  {
    throw std::runtime_error("No ros2_canopen driver registered for configured POSITAL node");
  }
  auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(driver_it->second);
  proxy_driver->register_rpdo_cb([this](ros2_canopen::COData data, std::uint8_t id)
                                 { on_rpdo(data, id); });
  proxy_driver->register_nmt_state_cb([this](canopen::NmtState state, std::uint8_t id)
                                      { on_nmt(state, id); });
  proxy_driver->register_emcy_cb([this](ros2_canopen::COEmcy emcy, std::uint8_t id)
                                 { on_emcy(emcy, id); });
}

void PositalHardwareInterface::on_rpdo(const ros2_canopen::COData data, const std::uint8_t node_id)
{
  if (node_id != node_id_ || data.index_ != kSlopeLong16Index || data.subindex_ != 0)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(sample_write_mutex_);
  non_rt_sample_.raw = static_cast<std::uint16_t>(data.data_ & 0xFFFFU);
  non_rt_sample_.received_at = std::chrono::steady_clock::now();
  non_rt_sample_.received = true;
  sample_buffer_.writeFromNonRT(non_rt_sample_);
}

void PositalHardwareInterface::on_nmt(const canopen::NmtState state, const std::uint8_t node_id)
{
  if (node_id != node_id_)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(sample_write_mutex_);
  non_rt_sample_.nmt_state = static_cast<std::uint8_t>(state);
  sample_buffer_.writeFromNonRT(non_rt_sample_);
}

void PositalHardwareInterface::on_emcy(const ros2_canopen::COEmcy emcy, const std::uint8_t node_id)
{
  if (node_id != node_id_)
  {
    return;
  }
  std::uint16_t error_code = 0;
  std::memcpy(&error_code, &emcy.eec, sizeof(error_code));
  std::lock_guard<std::mutex> lock(sample_write_mutex_);
  non_rt_sample_.emcy_error_code = error_code;
  sample_buffer_.writeFromNonRT(non_rt_sample_);
}

}  // namespace rebuild_posital

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rebuild_posital::PositalHardwareInterface,
                       hardware_interface::SensorInterface)
