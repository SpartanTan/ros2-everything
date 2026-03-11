# Nav2 Framework Pipeline

This note summarizes the main execution path in Nav2 from the top-level
`NavigateToPose` request down to `FollowPath` and the controller plugin.

The goal is to distinguish these layers clearly:

- `bt_navigator`: the host ROS 2 node
- `Navigator` plugin: task-level plugin such as `NavigateToPoseNavigator`
- `BtActionServer<ActionT>`: wraps a ROS 2 action server and a behavior tree
- BT XML: orchestration logic
- BT action node plugin: such as `ComputePathToPoseAction` or `FollowPathAction`
- downstream server: such as `planner_server` or `controller_server`
- algorithm plugin inside a server: such as DWB / RPP / MPPI controller plugins

## Big Picture

Typical call chain:

```text
External client / RViz
-> navigate_to_pose action server
-> NavigateToPoseNavigator
-> BtActionServer<NavigateToPose>
-> execute BT XML
-> ComputePathToPose BT node
-> planner_server
-> FollowPath BT node
-> controller_server
-> selected controller plugin
-> cmd_vel
```

## Layered Architecture

### 1. `bt_navigator`

Files:

- `src/navigation2/nav2_bt_navigator/src/bt_navigator.cpp`
- `src/navigation2/nav2_bt_navigator/include/nav2_bt_navigator/bt_navigator.hpp`

Role:

- A ROS 2 lifecycle node.
- Acts as the host for navigator plugins.
- Loads plugins whose base type is `nav2_core::NavigatorBase`.

Key facts:

- Registered as a component:

```cpp
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_bt_navigator::BtNavigator)
```

- Owns:
  - `pluginlib::ClassLoader<nav2_core::NavigatorBase> class_loader_`
  - `std::vector<pluginlib::UniquePtr<nav2_core::NavigatorBase>> navigators_`

- Default navigator plugins:
  - `nav2_bt_navigator::NavigateToPoseNavigator`
  - `nav2_bt_navigator::NavigateThroughPosesNavigator`

At configure time it loads navigator plugins:

```cpp
navigators_.push_back(class_loader_.createUniqueInstance(navigator_type));
navigators_.back()->on_configure(...);
```

At activate time it activates each navigator plugin:

```cpp
navigators_[i]->on_activate();
```

Important:

- `bt_navigator` is not itself the `NavigateToPose` action server implementation.
- It is the host node that loads navigator plugins that expose task-level actions.

### 2. `NavigatorBase` and `BehaviorTreeNavigator<ActionT>`

File:

- `src/navigation2/nav2_core/include/nav2_core/behavior_tree_navigator.hpp`

There are two related base classes in this file:

- `NavigatorBase`
  - non-template plugin interface
  - used by `pluginlib`
- `BehaviorTreeNavigator<ActionT>`
  - template helper base class
  - implements most common logic for BT-based navigation actions

Why both exist:

- `pluginlib` needs a non-template base interface to load heterogeneous plugins.
- the navigation task itself is templated on the action type, such as:
  - `nav2_msgs::action::NavigateToPose`
  - `nav2_msgs::action::NavigateThroughPoses`

`BehaviorTreeNavigator<ActionT>` is the shared framework for:

- creating `BtActionServer<ActionT>`
- loading the default BT XML
- owning the BT blackboard
- wiring high-level callbacks:
  - `onGoalReceived`
  - `onLoop`
  - `onPreempt`
  - `onCompletion`

Important distinction:

- the derived plugin does not override `on_configure()`
- it overrides `configure()`

`BehaviorTreeNavigator<ActionT>::on_configure()` is `final` and internally calls:

```cpp
return configure(parent_node, odom_smoother) && ok;
```

This is a template-method pattern:

- base class handles generic setup
- derived class adds task-specific setup

### 3. `NavigateToPoseNavigator`

Files:

- `src/navigation2/nav2_bt_navigator/include/nav2_bt_navigator/navigators/navigate_to_pose.hpp`
- `src/navigation2/nav2_bt_navigator/src/navigators/navigate_to_pose.cpp`

Role:

- Navigator plugin for the task type `nav2_msgs::action::NavigateToPose`
- Inherits:

```cpp
nav2_core::BehaviorTreeNavigator<nav2_msgs::action::NavigateToPose>
```

It provides task-specific logic:

- `getName() -> "navigate_to_pose"`
- default BT XML path
- handling incoming navigation goals
- publishing feedback
- optional `/goal_pose` topic bridge

#### `configure()`

This is the derived-class extension point. It does not create the main action
server. The main action server is created by the base class.

What it does create:

- `self_client_`
  - helper action client targeting `"navigate_to_pose"`
- `goal_sub_`
  - a topic subscription to `"goal_pose"`

Purpose of `self_client_`:

- if a pose arrives on `/goal_pose`, it converts it into a `NavigateToPose`
  action goal and sends it to the same `navigate_to_pose` action server

Code path:

```cpp
ActionT::Goal goal;
goal.pose = *pose;
self_client_->async_send_goal(goal);
```

This is only a convenience bridge for topic-based goal submission.

It is not required for the normal action path.

#### `goalReceived()`

This is the key callback after a `NavigateToPose` goal is accepted.

Responsibilities:

- optionally choose BT XML from the goal message
- load the BT XML into `BtActionServer`
- transform and validate the goal pose
- write the final goal pose into the blackboard

Key effect:

```cpp
blackboard->set(goal_blackboard_id_, goal_pose);
```

By default, `goal_blackboard_id_` is `"goal"`.

That value is later consumed by the BT XML:

```xml
<ComputePathToPose goal="{goal}" ... />
```

### 4. `BtActionServer<ActionT>`

Files:

- `src/navigation2/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server.hpp`
- `src/navigation2/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp`

Role:

- Wraps a ROS 2 action server
- Owns the behavior tree engine, blackboard, and current BT tree
- Executes the BT whenever an action goal is received

Owns:

- `std::shared_ptr<ActionServer> action_server_`
  - where `ActionServer = nav2_util::SimpleActionServer<ActionT>`
- `BT::Tree tree_`
- `BT::Blackboard::Ptr blackboard_`
- `std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_`

Constructor inputs include the high-level callbacks:

- `OnGoalReceivedCallback`
- `OnLoopCallback`
- `OnPreemptCallback`
- `OnCompletionCallback`

These are not ROS 2 native action callbacks.

They are Nav2 business-level callbacks used after the ROS action goal has
already been accepted by the underlying action server.

#### `executeCallback()`

This is the core execution function.

Flow:

1. call `on_goal_received_callback_(current_goal)`
2. run the behavior tree:

```cpp
bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);
```

3. when the BT finishes, complete the action result
4. call `on_completion_callback_(result, rc)`

So the BT is not always running.

It is run per accepted action goal.

### 5. `SimpleActionServer<ActionT>`

File:

- `src/navigation2/nav2_util/include/nav2_util/simple_action_server.hpp`

Role:

- thin Nav2 wrapper around `rclcpp_action::create_server<ActionT>(...)`
- handles:
  - `handle_goal`
  - `handle_cancel`
  - `handle_accepted`

This is the level that directly interfaces with ROS 2 action transport.

In `handle_accepted`, it starts a new thread to run `execute_callback_()`
```c++
execution_future_ = std::async(
  std::launch::async, [this]() {
    setSoftRealTimePriority();
    work();
  });

work()
{
try {
      execute_callback_();
    }
}
```
Where `execute_callback_` is provided when `SimpleActionServer` is consturcted in `BtActionServer`:
```c++
explicit SimpleActionServer(
    ...
    ExecuteCallback execute_callback,
    CompletionCallback completion_callback = nullptr,
    ...)
  : ...
    execute_callback_(execute_callback),
    completion_callback_(completion_callback),
```
Code piece in `BtActionServer`:
```c++
action_server_ = std::make_shared<ActionServer>(
  node->get_node_base_interface(),
  node->get_node_clock_interface(),
  node->get_node_logging_interface(),
  node->get_node_waitables_interface(),
  action_name_, std::bind(&BtActionServer<ActionT>::executeCallback, this),
  nullptr, std::chrono::milliseconds(500), false, server_options);
```

Relationship to `BtActionServer`:

- `BtActionServer` is built on top of `SimpleActionServer`
- `SimpleActionServer` handles ROS 2 action protocol
- `BtActionServer` handles business logic and BT execution

## From `NavigateToPose` Goal to Behavior Tree

There are two goal entry paths:

### Path A: external action client

```text
external client
-> navigate_to_pose action server
-> BtActionServer<NavigateToPose>::executeCallback()
```

### Path B: `/goal_pose` topic

```text
/goal_pose topic
-> NavigateToPoseNavigator::onGoalPoseReceived()
-> self_client_->async_send_goal(...)
-> navigate_to_pose action server
-> BtActionServer<NavigateToPose>::executeCallback()
```

After both paths merge:

1. `goalReceived()` loads BT XML
2. `initializeGoalPose()` writes blackboard `"goal"`
3. `BtActionServer::executeCallback()` runs the tree

## Behavior Tree XML

Typical file:

- `src/navigation2/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`

Important nodes in the default path:

```xml
<ComputePathToPose goal="{goal}" path="{path}" planner_id="{selected_planner}" ... />
<FollowPath path="{path}" controller_id="{selected_controller}" ... />
```

Interpretation:

- `{goal}` comes from the blackboard entry written by `NavigateToPoseNavigator`
- `ComputePathToPose` writes `{path}` to the blackboard
- `FollowPath` consumes `{path}`

This is the main bridge from top-level task goal to lower-level execution.

## BT Action Node Plugins

These are not navigator plugins.

They are BT node plugins used inside the XML.

Examples:

- `ComputePathToPoseAction`
- `FollowPathAction`
- `SpinAction`
- `BackUpAction`

### `ComputePathToPoseAction`

Files:

- `src/navigation2/nav2_behavior_tree/include/nav2_behavior_tree/plugins/action/compute_path_to_pose_action.hpp`
- `src/navigation2/nav2_behavior_tree/plugins/action/compute_path_to_pose_action.cpp`

Role:

- BT action node plugin
- wraps a ROS 2 action client of type `nav2_msgs::action::ComputePathToPose`

What `on_tick()` does:

- reads BT ports
- fills ROS action goal fields

Example:

```cpp
getInput("goal", goal_.goal);
getInput("planner_id", goal_.planner_id);
```

Then the base class `BtActionNode<ActionT>` sends that goal to the action server.

On success:

- writes the returned path to BT output port `path`
- that becomes blackboard `{path}` due to XML remapping

### `FollowPathAction`

Files:

- `src/navigation2/nav2_behavior_tree/include/nav2_behavior_tree/plugins/action/follow_path_action.hpp`
- `src/navigation2/nav2_behavior_tree/plugins/action/follow_path_action.cpp`

Role:

- BT action node plugin
- wraps a ROS 2 action client of type `nav2_msgs::action::FollowPath`

Critical distinction:

- BT node ID in XML: `FollowPath`
- ROS action name used by its client: `"follow_path"`
- controller plugin ID inside the goal: typically something like `"FollowPath"`

These are three different naming layers.

When plugin library is loaded by `BehaviorTreeEngine`
```c++
factory_.registerFromPlugin(...)
```

This part of code in `follow_path_action.cpp` will be executed:

```c++
BT_REGISTER_NODES(factory)
```

where,

```c++
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::FollowPathAction>(
        name, "follow_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::FollowPathAction>(
    "FollowPath", builder);
}
```
It means "If xml meets node ID "FollowPath", call the `builder`". 

```c++
(const std::string & name, const BT::NodeConfiguration & config)
```
The parameter `name` and `config` are prased by BT library when pluin is loaded by xml. 

```c++
std::make_unique<nav2_behavior_tree::FollowPathAction>(
  name, "follow_path", config);
```
The BT node's action client connects to the action server named
`"follow_path"`.





## `BtActionNode<ActionT>`: how a BT node sends an action goal

File:

- `src/navigation2/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp`

Role:

- base class used by BT action node plugins
- wraps a ROS 2 action client

Typical flow inside `tick()`:

1. if first entry:
   - clear old goal/result state
   - call derived `on_tick()`
   - call `send_new_goal()`
2. while running:
   - poll feedback/result
   - call `on_wait_for_result(...)` where `async_send_goal()` is called 
3. on completion:
   - call one of:
     - `on_success()`
     - `on_aborted()`
     - `on_cancelled()`

Important:

- derived `on_tick()` only fills `goal_`
- the base class actually sends the action goal via:

```cpp
action_client_->async_send_goal(goal_, send_goal_options);
```

## `controller_server`

Files:

- `src/navigation2/nav2_controller/src/controller_server.cpp`
- `src/navigation2/nav2_controller/include/nav2_controller/controller_server.hpp`

Role:

- ROS 2 node that implements the `nav2_msgs::action::FollowPath` action server
- action server name: `"follow_path"`

This is the downstream server that receives goals from `FollowPathAction`.

Its action server is created with:

```cpp
action_server_ = std::make_unique<ActionServer>(
  shared_from_this(),
  "follow_path",
  std::bind(&ControllerServer::computeControl, this),
  ...);
```

### What happens when a FollowPath goal arrives

Inside `computeControl()`:

1. read current action goal:

```cpp
auto goal = action_server_->get_current_goal();
```

2. extract:
   - `goal->path`
   - `goal->controller_id`
   - `goal->goal_checker_id`
   - `goal->progress_checker_id`

3. select the requested controller plugin:

```cpp
findControllerId(goal->controller_id, current_controller);
```

4. pass the path into that controller plugin:

```cpp
controllers_[current_controller_]->setPlan(path);
```

5. loop and repeatedly compute velocity:

```cpp
controllers_[current_controller_]->computeVelocityCommands(...)
```

6. publish `cmd_vel`

So the controller plugin is not a standalone ROS node.

The continuously running node is `controller_server`.

The controller plugin is a loaded algorithm object inside that node.

## Why controller is a plugin, not a separate node

Nav2 chooses this design so that:

- multiple control algorithms share one action server
- switching controller is just selecting a plugin ID
- costmap / tf / odom / cmd_vel publishing stay centralized
- less inter-node communication is required

This means:

- `controller_server` is the active server node
- DWB / RPP / MPPI are plugins loaded into that node

## Naming Cheat Sheet

This is one of the most common sources of confusion.

### Navigator plugin name

Examples:

- `"navigate_to_pose"`
- `"navigate_through_poses"`

Used by:

- `BtActionServer<ActionT>` created by `BehaviorTreeNavigator`
- top-level action name exposed to external clients

### BT XML node ID

Examples:

- `ComputePathToPose`
- `FollowPath`

Used by:

- BehaviorTree factory when instantiating BT nodes from XML

### ROS action name

Examples:

- `"compute_path_to_pose"`
- `"follow_path"`

Used by:

- ROS action client/server transport matching

### Controller plugin ID

Examples:

- `"FollowPath"`
- `"RPP"`
- `"MPPI"`

Used by:

- `controller_server` to choose which loaded controller plugin instance to use

## Main Execution Timeline

Compact version:

```text
1. lifecycle manager activates bt_navigator
2. bt_navigator activates navigator plugins
3. NavigateToPose navigator exposes action server "navigate_to_pose"
4. external client or /goal_pose sends a NavigateToPose goal
5. BtActionServer receives the goal
6. NavigateToPoseNavigator::goalReceived() loads XML and writes blackboard["goal"]
7. BtActionServer runs the tree
8. ComputePathToPose sends action goal to planner_server
9. planner result path is written to blackboard["path"]
10. FollowPath sends action goal to controller_server ("follow_path")
11. controller_server selects controller plugin and computes cmd_vel
12. tree completes, BtActionServer completes NavigateToPose action result
```

## Recommended Reading Order in Source

If you want to re-walk the architecture in code, this order is the most useful:

1. `nav2_bt_navigator/src/bt_navigator.cpp`
2. `nav2_core/include/nav2_core/behavior_tree_navigator.hpp`
3. `nav2_bt_navigator/src/navigators/navigate_to_pose.cpp`
4. `nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server.hpp`
5. `nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp`
6. `nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`
7. `nav2_behavior_tree/plugins/action/compute_path_to_pose_action.cpp`
8. `nav2_behavior_tree/plugins/action/follow_path_action.cpp`
9. `nav2_controller/src/controller_server.cpp`

## Common Confusions

### `NavigateToPoseNavigator` does not override `on_configure()`

It overrides `configure()`.

`BehaviorTreeNavigator::on_configure()` is `final` and calls the derived
`configure()`.

### `NavigateToPose` is itself an action server

Yes.

It is the top-level navigation task action.

### `FollowPath` is not above `NavigateToPose`

`FollowPath` is a lower-level action used inside the behavior tree that
implements `NavigateToPose`.

### `controller_id` is not the action server name

`controller_id` selects the internal controller plugin inside
`controller_server`.

The action server name for the BT node client/server transport is
`"follow_path"`.
