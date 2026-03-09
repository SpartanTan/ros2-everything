#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

struct Position2D
{
  double x;
  double y;
};

using namespace BT;

namespace BT
{
  // this conversion is called when 
  // getInput<Position2D>("target"), wehere target="{OtherGoal} is "-1;3"
  // this will trigger the conversion below
  template <>
  inline Position2D convertFromString(StringView str)
  {
    auto parts = splitString(str, ';');
    if (parts.size() != 2)
    {
      throw RuntimeError("invalid input)");
    }
    else
    {
      Position2D output;
      output.x = convertFromString<double>(parts[0]);
      output.y = convertFromString<double>(parts[1]);
      printf("Converting string: [ %.1f, %.1f ]\n", output.x, output.y);
      return output;
    }
  }
}

class CalculateGoal : public SyncActionNode
{
public:
  CalculateGoal(const std::string &name, const NodeConfig &config) : SyncActionNode(name, config)
  {
  }

  static PortsList providedPorts()
  {
    return {OutputPort<Position2D>("goal")};
  }
  NodeStatus tick() override
  {
    Position2D mygoal = {1.1, 2.3};
    setOutput<Position2D>("goal", mygoal);
    return NodeStatus::SUCCESS;
  }
};

class PrintTarget : public SyncActionNode
{
public:
  PrintTarget(const std::string &name, const NodeConfig &config) : SyncActionNode(name, config)
  {
  }

  static PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char *description = "Simply print the goal on console...";
    return {InputPort<Position2D>("target", description)};
  }

  NodeStatus tick() override
  {
    auto res = getInput<Position2D>("target");
    if (!res)
    {
      throw RuntimeError("error reading port [target]:", res.error());
    }
    Position2D target = res.value();
    printf("Target positions: [ %.1f, %.1f ]\n", target.x, target.y);
    return NodeStatus::SUCCESS;
  }
};

int main()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CalculateGoal>("CalculateGoal");
  factory.registerNodeType<PrintTarget>("PrintTarget");

  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("bt_sample");
  auto tree = factory.createTreeFromFile(pkg_share_dir + "/trees/generic_ports.xml");

  tree.tickWhileRunning();

  return 0;
}