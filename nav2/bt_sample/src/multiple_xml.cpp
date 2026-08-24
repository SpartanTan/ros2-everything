#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <chrono>

int main()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<DummyNodes::SaySomething>("SaySomething");
  std::string searching_directory = "./";
  using std::filesystem::directory_iterator;

  // find all .xml files in current dir
  for (auto const& entry : directory_iterator(search_directory)) 
  {
    if( entry.path().extension() == ".xml")
    {
      factory.registerBehaviorTreeFromFile(entry.path().string());
    }
  }

  // alternatively, using one .xml which has include the dependencies
  factory.createTreeFromFile("main_tree_include.xml");


  return 0;
}