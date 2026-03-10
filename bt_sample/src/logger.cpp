#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_observer.h>

#include <chrono>
#include <map>
#include <rclcpp/rclcpp.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

struct NodeStatistics
{
  BT::NodeStatus last_result;
  BT::NodeStatus current_status;
  unsigned transitions_count;
  unsigned success_count;
  unsigned failure_count;
  unsigned skip_count;
  BT::Duration last_timestamp;
};

int main()
{
  BT::BehaviorTreeFactory factory;
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("bt_sample");
  auto tree = factory.createTreeFromFile(pkg_share_dir + "/trees/logger.xml");

  BT::printTreeRecursively(tree.rootNode());

  BT::TreeObserver observer(tree);

  std::map<uint16_t, std::string> ordered_UID_to_path;

  for (const auto & [name, uid] : observer.pathToUID()) {
    ordered_UID_to_path[uid] = name;
  }

  for (const auto & [uid, name] : ordered_UID_to_path) {
    std::cout << uid << " -> " << name << std::endl;
  }

  tree.tickWhileRunning();

  // You can access a specific statistic, using is full path or the UID
  const auto & last_action_stats = observer.getStatistics("last_action");
  assert(last_action_stats.transitions_count > 0);

  std::cout << "----------------" << std::endl;
  // print all the statistics
  for (const auto & [uid, name] : ordered_UID_to_path) {
    const auto & stats = observer.getStatistics(uid);

    std::cout << "[" << name << "] \tT/S/F:  " << stats.transitions_count << "/"
              << stats.success_count << "/" << stats.failure_count << std::endl;
  }

  return 0;
}