#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_observer.h>

#include <chrono>
#include <map>
#include <rclcpp/rclcpp.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

int main()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SaySomething>("SaySomething");

  factory.registerSimpleAction("DummyAction", [](BT::TreeNode & self) {
    std::cout << "DummyAction substituting: " << self.name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  });

  factory.registerSimpleAction("TestSaySomething", [](BT::TreeNode & self) {
    auto msg = self.getInput<std::string>("message");
    if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }
    std::cout << "TestSaySomething: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  });

  bool skip_substitution = (argc == 2) && std::string(argv[1]) == "no_sub";

  if (!skip_substitution) {
    bool const USE_JSON = true;
    if (USE_JSON) {
      factory.loadSubstitutionRuleFromJSON(json_text);
    } else {
      // Substitute nodes which match this wildcard pattern with TestAction
      factory.addSubstitutionRule("mysub/action_*", "TestAction");

      // Substitute the node with name [talk] with TestSaySomething
      factory.addSubstitutionRule("talk", "TestSaySomething");

      // This configuration will be passed to a TestNode
      BT::TestNodeConfig test_config;

      // Convert the node in asynchronous and wait 2000 ms
      test_config.async_delay = std::chrono::milliseconds(2000);

      // Execute this postcondition, once completed
      test_config.post_script = "msg ='message SUBSTITUED'";

      // Substitute the node with name [last_action] with a TestNode,
      // configured using test_config
      factory.addSubstitutionRule("last_action", test_config);
    }
  }

  factory.registerBehaviorTreeFromText(xml_text);

  auto tree = factory.createTree("MainTree");
  tree.tickWhileRunning();

  return 0;
}