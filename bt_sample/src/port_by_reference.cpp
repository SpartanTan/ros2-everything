#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_observer.h>

#include <chrono>
#include <map>
#include <rclcpp/rclcpp.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

// using shared_ptr
// not thread-safe
BT::PortsList AcquirePointCloud::providePorts()
{
  return { BT::OutputPort<std::shared_ptr<PointCloud>>("cloud"); }
}

PortsList SegmentObject::providedPorts()
{
  return {
    InputPort<std::string>("obj_name"), InputPort<std::shared_ptr<Pointcloud>>("cloud"),
    OutputPort<Pose3D>("obj_pose")};
}

// thread-safe castPtr
// simply use plain <Pointcloud> when creating ports
BT::PortsList AcquirePointCloud::providedPorts() { return {OutputPort<Pointcloud>("cloud")}; }

BT::PortsList SegmentObject::providedPorts()
{
  return {
    InputPort<std::string>("obj_name"), InputPort<Pointcloud>("cloud"),
    OutputPort<Pose3D>("obj_pose")};
}

// access the instance by pointer/reference
if (auto any_locked = BT::getLockedPortContent("cloud")) {
  if (any_locked->empty()) {
    // the entry in the blackboard hasn't been initialized yet.
    // You can initialize it doing:
    any_locked.assign(my_initial_pointcloud);
  }
  else if (Pointcloud* cloud_ptr = any_locked->castPtr<Pointcloud>())
  {
    // successfully cast to Pointcloud*
    // modify the pointcloud instance with cloud_ptr
  }
}