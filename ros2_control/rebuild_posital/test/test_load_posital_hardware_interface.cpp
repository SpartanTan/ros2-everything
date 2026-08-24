#include <hardware_interface/sensor_interface.hpp>
#include <pluginlib/class_loader.hpp>

#include <gtest/gtest.h>

TEST(LoadPositalHardwarePlugin, can_be_found_and_created)
{
  pluginlib::ClassLoader<hardware_interface::SensorInterface> loader(
      "hardware_interface", "hardware_interface::SensorInterface");
  constexpr auto kPluginName = "rebuild_posital/PositalHardwareInterface";

  ASSERT_TRUE(loader.isClassAvailable(kPluginName));
  auto hardware = loader.createSharedInstance(kPluginName);

  ASSERT_NE(hardware, nullptr);
}