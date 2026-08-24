#include <behaviortree_cpp/bt_factory.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono;

class SleepNode : public BT::StatefulActionNode
{
public:
  SleepNode(const std::string & name, BT::NodeConfig & config) : StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {BT::InputPort<int>("msec")}; }

  BT::NodeStatus onStart() override
  {
    int msec = 0;
    getInput("msec", msec);

    if (msec <= 0) {
      return BT::NodeStatus::SUCCESS;
    } else {
      deadline_ = system_clock::now() + milliseconds(msec);
      return BT::NodeStatus::RUNNING;
    }
  }

  BT::NodeStatus onRunning() override
  {
    if (system_clock::now() >= deadline_) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::RUNNING;
    }
  }

  void onHalted() override { std::cout << "SleepNode interrupted" << std::endl; }

private:
  system_clock::time_point deadline_;
};