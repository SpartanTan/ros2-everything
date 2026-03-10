#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class FibonacciAction : public nav2_behavior_tree::BtActionNode<Fibonacci>
{
public:
  FibonacciAction(
    const std::string & xml_tag_name, std::string & action_name, const BT::NodeConfiguration & conf)
  : BtActionNode<Fibonacci>(xml_tag_name, action_name, conf)
  {
  }
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<unsigned>("order")});
  }

  bool on_tick(RosActionNode::Goal & goal) override
  {
    // get "order" from the Input port
    getInput("order", goal.order);
    // return true, if we were able to set the goal correctly.
    return true;
  }
};