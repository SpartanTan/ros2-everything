#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_observer.h>

#include <chrono>
#include <map>
#include <rclcpp/rclcpp.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

class PrintNumber : public BT::SyncActionNode
{
public:
  PrintNumber(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {BT::InputPort<int>("val")}; }

  BT::NodeStatus tick() override
  {
    const int val = getInput<int>("val").value();
    std::cout << "[" << name() << "] val: " << val << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

int main()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<PrintNumber>("PrintNumber");
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("bt_sample");
  factory.registerBehaviorTreeFromFile(pkg_share_dir + "/trees/global_blackboard.xml");

  auto global_bb = BT::Blackboard::create();
  auto maintree_bb = BT::Blackboard::create(global_bb);
  auto tree = factory.createTree("MainTree", maintree_bb);

  // interact directly with global_bb
  for (int i = 1; i <= 3; i++) {
    global_bb->set("value", i);

    // tick the tree
    tree.tickOnce();

    // read the entry "value_sqr"
    auto value_sqr = global_bb->get<int>("value_sqr");

    // print
    std::cout << "[While loop] value: " << i << " value_sqr: " << value_sqr << "\n\n";
  }

  return 0;
}