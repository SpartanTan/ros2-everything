#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <chrono>

class CrossDoor
{
public:
  void registerNodes(BT::BehaviorTreeFactory &factory);

  // SUCCESS if _door_open != true
  BT::NodeStatus isDoorClosed();

  // SUCCESS if _door_open == true
  BT::NodeStatus passThroughDoor();

  // After 3 attempts, will open a locked door
  BT::NodeStatus pickLock();

  // FAILURE if door locked
  BT::NodeStatus openDoor();

  // WILL always open a door
  BT::NodeStatus smashDoor();

private:
  bool _door_open = false;
  bool _door_locked = true;
  int _pick_attempts = 0;
};

void CrossDoor::registerNodes(BT::BehaviorTreeFactory &factory)
{
  factory.registerSimpleCondition("IsDoorClosed", std::bind(&CrossDoor::isDoorClosed, this));
  factory.registerSimpleAction("PassThroughDoor", std::bind(&CrossDoor::passThroughDoor, this));
  factory.registerSimpleAction("OpenDoor", std::bind(&CrossDoor::openDoor, this));
  factory.registerSimpleAction("PickLock", std::bind(&CrossDoor::pickLock, this));
  factory.registerSimpleCondition("SmashDoor", std::bind(&CrossDoor::smashDoor, this));
}

int main()
{
  BT::BehaviorTreeFactory factory;
  CrossDoor cross_door;
  cross_door.registerNodes(factory);

  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("bt_sample");
  auto tree = factory.createTreeFromFile(pkg_share_dir + "/trees/sub_trees.xml");

  BT::printTreeRecursively(tree.rootNode());
  tree.tickWhileRunning();
  return 0;
}