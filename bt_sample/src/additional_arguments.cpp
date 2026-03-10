#include <behaviortree_cpp/bt_factory.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

class Action_A : public BT::SyncActionNode
{
public:
  Action_A(
    const std::string & name, const BT::NodeConfig & config, int arg_int, std::string arg_str)
  : SyncActionNode(name, config), _arg1(arg_int), _arg2(arg_str)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  int _arg1;
  std::string _arg2;
};

// if different values needed
class Action_B : public BT::SyncActionNode
{
public:
  Action_B(const std::string & name, const BT::NodeConfig & config) : SyncActionNode(name, config)
  {
  }

  void initialize(int arg_int, const std::string & arg_str)
  {
    _arg1 = arg_int;
    _arg2 = arg_str;
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  int _arg1;
  std::string _arg2;
};

int main()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<Action_A>("Action_A", 42, "hello_world");

  factory.registerNodeType<Action_B>("Action_B");

  auto tree = factory.createTreeFromText(xml_text);

  auto visitor = [](BT::TreeNode * node) {
    if (auto action_B_node = dynamic_cast<Action_B *>(node)) {
      action_B_node->initialize(69, "interesting_value");
    }
  };

  tree.applyVisitor(visitor);
}