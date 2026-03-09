#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <chrono>

struct Pose2D
{
  double x, y, theta;
};

namespace BT
{
  // this conversion is called when
  // getInput<Position2D>("target"), wehere target="{OtherGoal} is "-1;3"
  // this will trigger the conversion below
  template <>
  inline Pose2D convertFromString(StringView str)
  {
    auto parts = splitString(str, ';');
    if (parts.size() != 3)
    {
      throw RuntimeError("invalid input)");
    }
    else
    {
      Pose2D output;
      output.x = convertFromString<double>(parts[0]);
      output.y = convertFromString<double>(parts[1]);
      output.theta = convertFromString<double>(parts[1]);

      printf("Converting string: [ %.1f, %.1f, %.1f ]\n", output.x, output.y, output.theta);
      return output;
    }
  }
}

namespace chr = std::chrono;

BT::NodeStatus CheckBattery()
{
  std::cout << "[ Battery: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

class SaySomething : public BT::SyncActionNode
{
public:
  SaySomething(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("message")};
  }

  BT::NodeStatus tick() override
  {
    BT::Expected<std::string> msg = getInput<std::string>("message");
    if (!msg)
    {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }

    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

class MoveBaseAction : public BT::StatefulActionNode
{
public:
  MoveBaseAction(const std::string &name, const BT::NodeConfig &config) : StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Pose2D>("goal")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  Pose2D _goal;
  chr::system_clock::time_point _completion_time;
};

BT::NodeStatus MoveBaseAction::onStart()
{
  if (!getInput<Pose2D>("goal", _goal))
  {
    throw BT::RuntimeError("missing required input [goal]");
  }
  printf("[ MoveBase: SEND REQUEST ]. goal: x=%f y=%f theta=%f\n",
         _goal.x, _goal.y, _goal.theta);
  _completion_time = chr::system_clock::now() + chr::milliseconds(220);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveBaseAction::onRunning()
{
  std::this_thread::sleep_for(chr::milliseconds(10));
  if (chr::system_clock::now() >= _completion_time)
  {
    std::cout << "[ MoveBase: FINISHED ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void MoveBaseAction::onHalted()
{
  printf("[ MoveBase: ABORTED ]");
}

int main()
{
  BT::BehaviorTreeFactory factory;
  factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
  factory.registerNodeType<MoveBaseAction>("MoveBase");
  factory.registerNodeType<SaySomething>("SaySomething");

  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("bt_sample");
  auto tree = factory.createTreeFromFile(pkg_share_dir + "/trees/reactive_behaviors.xml");

  std::cout << "--- ticking\n";
  auto status = tree.tickOnce();
  std::cout << "--- status: " << toStr(status) << "\n\n";

  while (status == BT::NodeStatus::RUNNING)
  {
    tree.sleep(std::chrono::milliseconds(100));
    std::cout << "--- ticking\n";
    status = tree.tickOnce();
    std::cout << "--- status: " << toStr(status) << "\n\n";
  }
  return 0;
}