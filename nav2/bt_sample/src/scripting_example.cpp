#include <behaviortree_cpp/bt_factory.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

enum Color
{
  RED = 1,
  BLUE = 2,
  GREEN = 3
};

int main()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<DummyNodes::SaySomething>("SaySomething");

  // We can add these enums to the scripting language.
  // Check the limits of magic_enum
  factory.registerScriptingEnums<Color>();

  // Or we can manually assign a number to the label "THE_ANSWER".
  // This is not affected by any range limitation
  factory.registerScriptingEnum("THE_ANSWER", 42);

  auto tree = factory.createTreeFromText(xml_text);
  tree.tickWhileRunning();

  return 0;
}