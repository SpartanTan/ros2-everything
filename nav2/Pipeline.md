# Nav2 framework pipeline

## bt_navigator
`bt_navigator.cpp`, `bt_navigator.hpp`

Implenment `nav2_bt_navigator::BtNavigator`, inherit from `nav2_util::LifecycleNode`
Run as component:
```c++
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_bt_navigator::BtNavigator)
```
Owns `class_loader_` to dynamically load modules of base class `nav2_core::NavigatorBase`, for example  `NavigateToPoseNavigator` which is the child of `nav2_core::BehaviorTreeNavigator`, which inherited from `nav2_core::NavigatorBase`. See [NavigateToPoseNavigator](#navigatetoposenavigator)


## NavigateToPoseNavigator
`navigate_to_pose.cpp`, `navigate_to_pose.hpp`

Implenment a plugin `NavigateToPoseNavigator` of type `nav2_core::NavigatorBase`.

As mentioned before, `NavigateToPoseNavigator` inherits from `nav2_core::BehaviorTreeNavigator` which inherits from `nav2_core::NavigatorBase`, check [BehaviorTreeNavigator](#behaviortreenavigator) 

- `configure()`
  ```c++
  // create helper client to send request when receiving "goal_pose" message
  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  // helper subscriber to receive msg from "goal_pose"
  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose",
      std::bind(onGoalPoseReceived));
  ```
- <a id="goalReceived"></a>`goalReceived`
  ```c++
  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename))
  ```
### BehaviorTreeNavigator
`behavior_tree_navigator.hpp`
**nav2_core::BehaviorTreeNavigator**  
- `on_configure()`
  ```c++
  bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
    ...
  onGoalReceived,
  onLoop,
  onPreempt,
  onCompletion);

  // run configure() implenmented in child class
  return configure(parent_node, odom_smoother) && ok;
  ```
  Create [BtActionServer](#btactionserver) with type `ActionT`. Register the callback functions.

- `onGoalReceived`
  ```c++
  bool goal_accepted = goalReceived(goal);
  if (goal_accepted) {
    plugin_muxer_->startNavigating(getName());
  }
  ```
  `goalReceived()` is implenmented in child class, for example [goalReceived](#goalReceived)
## BtActionServer
`bt_action_server.hpp` and `bt_action_server_impl.hpp`

Implenment template class `BtActionServer`, owns `std::shared_ptr<ActionServer> action_server_;` where `ActionServer = nav2_util::SimpleActionServer<ActionT>`, check [SimpleActionServer](#simpleactionserver)

Owns
- std::shared_ptr<ActionServer> action_server_;
- BT::Tree tree_;
- BT::Blackboard::Ptr blackboard_;
-  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;


### SimpleActionServer
`simple_action_server.hpp`

Here you have the final `action_server_` implenmentation
```
typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
```
And in its constructor
```c++
action_server_ = rclcpp_action::create_server<ActionT>(
  node_base_interface_,
  node_clock_interface_,
  node_logging_interface_,
  node_waitables_interface_,
  action_name_,
  std::bind(handle_goal),
  std::bind(handle_cancel),
  std::bind(handle_accepted),
  options,
  callback_group_);
```