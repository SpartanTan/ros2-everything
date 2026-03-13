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

Important summary:

- `bt_navigator` does not directly instantiate `BehaviorTreeNavigator`
  itself.
- `bt_navigator` loads plugins of base type `NavigatorBase`.
- one concrete plugin is `NavigateToPoseNavigator`
- `NavigateToPoseNavigator` inherits from:

```cpp
BehaviorTreeNavigator<nav2_msgs::action::NavigateToPose>
```

- therefore its action type is `nav2_msgs::action::NavigateToPose`

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

For `NavigateToPoseNavigator`, this becomes:

```cpp
BtActionServer<nav2_msgs::action::NavigateToPose>
```

And this means:

- the navigator plugin owns a BT-aware action server
- that action server internally owns a `SimpleActionServer<nav2_msgs::action::NavigateToPose>`

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

### 4.1 How the top-level `navigate_to_pose` action server is created

This is one of the most important chains in the whole framework.

`BehaviorTreeNavigator<ActionT>::on_configure()` creates:

```cpp
bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
  node,
  getName(),
  ...
);
```

For `NavigateToPoseNavigator`, `ActionT` is:

```cpp
nav2_msgs::action::NavigateToPose
```

and `getName()` returns:

```cpp
"navigate_to_pose"
```

Therefore this chain is formed:

```text
NavigateToPoseNavigator
-> owns BtActionServer<nav2_msgs::action::NavigateToPose>
-> BtActionServer owns SimpleActionServer<nav2_msgs::action::NavigateToPose>
-> SimpleActionServer creates rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
-> action name is "navigate_to_pose"
```

So yes, the final top-level ROS 2 action server name comes from the hardcoded
`getName()` implementation in `NavigateToPoseNavigator`.

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

### 5.1 Where `handle_goal`, `handle_cancel`, and `handle_accepted` fit

These three methods are implemented in `SimpleActionServer`, not in
`NavigateToPoseNavigator`.

They are the native ROS 2 action server callbacks used by:

```cpp
rclcpp_action::create_server<ActionT>(...)
```

Flow:

- `handle_goal()`
  - accepts or rejects the goal
- `handle_cancel()`
  - accepts or rejects cancellation
- `handle_accepted()`
  - starts asynchronous execution

`handle_accepted()` does not directly tick the behavior tree itself.

Instead it starts a worker thread, and that worker thread eventually calls:

```cpp
execute_callback_()
```

For `BtActionServer`, `execute_callback_()` is:

```cpp
BtActionServer::executeCallback()
```

That is the actual entry point that then calls:

- `on_goal_received_callback_()`
- `bt_->run(...)`

So the BT is ultimately triggered from the accepted-goal path, but indirectly
through `SimpleActionServer -> work() -> execute_callback_() ->
BtActionServer::executeCallback()`.

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

At this point, the top-level `navigate_to_pose` action is active and remains in
the ROS 2 `RUNNING` / executing state while the BT is running.

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

## High-Level Summary

This is the most important compact mental model:

- a navigator plugin creates a top-level action server, such as
  `navigate_to_pose`
- after it receives a request, it starts running a behavior tree
- that behavior tree contains BT plugins such as `ComputePathToPose` and
  `FollowPath`
- those BT plugins read parameters from BT ports / blackboard
- then they send lower-level ROS 2 action requests through their internal
  action clients
- `ComputePathToPose` talks to the planner server
- `FollowPath` talks to the `follow_path` action server, which is implemented
  by `controller_server`
- `controller_server` then calls its internal controller algorithm plugins and
  publishes `cmd_vel`

This is the core Nav2 decomposition:

```text
top-level task action
-> behavior tree orchestration
-> lower-level action servers
-> internal algorithm plugins
-> robot command output
```

Another valid mental model:

- `NavigateToPoseNavigator` owns a top-level BT-aware action server
- that server receives `navigate_to_pose` goals
- after a goal is accepted, it runs a behavior tree
- the behavior tree may invoke BT plugins such as `FollowPath` or `Spin`
- those BT plugins package their inputs into lower-level ROS 2 action goals
- those lower-level goals are sent to task-specific servers such as:
  - `controller_server` for `follow_path`
  - `behavior_server` for `spin`, `wait`, `backup`, `drive_on_heading`

So the top-level task action is decomposed by the BT into lower-level action
requests.

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

This is why `FollowPath` can be thought of as similar to a long-running BT node
such as a custom `MoveBase` example in pure BehaviorTree.CPP, except that Nav2
implements it as a BT plugin wrapping a ROS 2 action client.

The same pattern applies to:

- `SpinAction`
- `WaitAction`
- `BackUpAction`
- `DriveOnHeadingAction`

These are BT plugins that wrap ROS 2 action clients, not the final behavior
algorithm implementations themselves.

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

## `behavior_server`

Files:

- `src/navigation2/nav2_behaviors/src/behavior_server.cpp`
- `src/navigation2/nav2_behaviors/src/main.cpp`

Role:

- ROS 2 node that hosts a set of behavior action servers
- loads `nav2_core::Behavior` plugins with `pluginlib`
- serves actions such as:
  - `spin`
  - `backup`
  - `drive_on_heading`
  - `wait`

It is launched as a standalone node, just like `bt_navigator`,
`controller_server`, and `planner_server`.

From launch:

- `navigation_launch.py` starts `behavior_server`
- `nav2_params.yaml` provides:
  - `behavior_plugins`
  - the concrete plugin class for each behavior ID

Example configuration:

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    wait:
      plugin: "nav2_behaviors::Wait"
```

At runtime, `behavior_server`:

1. reads `behavior_plugins`
2. loads the plugin classes with `pluginlib`
3. configures and activates them
4. exposes the corresponding ROS 2 actions

### `Behavior Plugin` vs `Behavior Tree Plugin`

This is a common source of confusion.

For `Spin`, there are two separate layers:

- BT plugin:
  - `nav2_behavior_tree::SpinAction`
  - this is what the XML node `<Spin .../>` maps to
  - wraps a ROS 2 action client
- Behavior plugin:
  - `nav2_behaviors::Spin`
  - loaded by `behavior_server`
  - contains the actual behavior implementation

So:

```text
XML <Spin .../>
-> BT plugin SpinAction
-> ROS 2 action goal sent to "spin"
-> behavior_server
-> nav2_behaviors::Spin
-> publishes cmd_vel while executing
```

The same structure applies to:

- `Wait`
- `BackUp`
- `DriveOnHeading`

### Example: `Spin`

Behavior tree XML:

```xml
<Spin spin_dist="1.57" error_code_id="{spin_error_code}"/>
```

BT plugin layer:

- `nav2_behavior_tree/plugins/action/spin_action.cpp`
- registers XML node ID `"Spin"`
- creates a `SpinAction` BT node
- uses ROS action name `"spin"`

Behavior plugin layer:

- `nav2_behaviors/plugins/spin.cpp`
- exported as `nav2_core::Behavior`
- loaded by `behavior_server`
- performs the actual spin behavior
- `execute()` method of parent class `TimedBehavior` binded into `SimpleActionServer`, will be called asyncly in `goal_handle` of `SimpleActionServer`

At execution time:

1. the BT ticks `SpinAction`
2. `SpinAction` reads BT inputs such as `spin_dist`
3. it fills a ROS action goal and sends it to action `"spin"`
4. `behavior_server` receives that goal
5. the corresponding behavior plugin `nav2_behaviors::Spin` executes
6. during execution, `execute()` calls methods such as `onCycleUpdate()` may publish velocity commands

So the XML node does not directly execute `nav2_behaviors::Spin`.

It executes `SpinAction`, which forwards the request to the behavior server.

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

Related recovery-action branch:

```text
BT recovery subtree
-> XML <Spin/> or <Wait/> or <BackUp/>
-> BT plugin SpinAction / WaitAction / BackUpAction
-> behavior_server action "spin" / "wait" / "backup"
-> nav2_behaviors::Spin / Wait / BackUp
-> cmd_vel or timed behavior execution
```

## Parallel Example Flows

These two examples are the best side-by-side comparison for understanding Nav2:

- `FollowPath`: lower-level path execution through `controller_server`
- `Spin`: recovery / utility behavior through `behavior_server`

### Flow A: `FollowPath`

```text
External NavigateToPose goal
-> navigate_to_pose action server
-> NavigateToPoseNavigator::goalReceived()
-> blackboard["goal"] = goal_pose
-> BT XML runs
-> <ComputePathToPose goal="{goal}" path="{path}" .../>
-> ComputePathToPoseAction
-> action client sends goal to "compute_path_to_pose"
-> planner_server
-> planner result writes blackboard["path"]
-> <FollowPath path="{path}" controller_id="{selected_controller}" .../>
-> FollowPathAction
-> action client sends goal to "follow_path"
-> controller_server
-> selects controller plugin by controller_id
-> controller plugin computes velocity
-> controller_server publishes cmd_vel
```

### Flow B: `Spin`

```text
External NavigateToPose goal
-> navigate_to_pose action server
-> NavigateToPoseNavigator::goalReceived()
-> behavior tree enters recovery branch
-> <Spin spin_dist="1.57" error_code_id="{spin_error_code}"/>
-> SpinAction
-> BT node reads spin_dist from BT ports / blackboard
-> action client sends goal to "spin"
-> behavior_server
-> behavior_server routes request to nav2_behaviors::Spin
-> nav2_behaviors::Spin executes behavior
-> onCycleUpdate() publishes motion commands as needed
-> behavior finishes and returns action result
-> SpinAction returns SUCCESS / FAILURE to the behavior tree
```

### Key Comparison

```text
FollowPath branch:
BT XML -> BT action plugin -> controller_server -> controller plugin

Spin branch:
BT XML -> BT action plugin -> behavior_server -> behavior plugin
```

So both branches share the same architectural shape:

```text
behavior tree
-> BT plugin
-> lower-level action server
-> server-internal plugin
```

The main difference is the downstream server and the plugin category:

- `FollowPath`
  - downstream server: `controller_server`
  - internal plugin type: `nav2_core::Controller`
- `Spin`
  - downstream server: `behavior_server`
  - internal plugin type: `nav2_core::Behavior`

## State of the top-level `navigate_to_pose` action while `FollowPath` runs

When `FollowPath` sends a request to `controller_server`, that lower-level
server may run for a long time while it continuously computes and publishes
velocity commands.

During that period:

- `FollowPathAction` remains in `RUNNING`
- the behavior tree remains in `RUNNING`
- `BtActionServer::executeCallback()` is still running
- the top-level `navigate_to_pose` action also remains active / running

This does not block the ROS executor thread because `SimpleActionServer`
launches execution asynchronously after `handle_accepted()`.

So:

- the work is asynchronous
- but the top-level action is still logically executing
- therefore its state remains running until the full BT completes

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

### `SpinAction` is not `nav2_behaviors::Spin`

- `SpinAction` is the BT XML-facing node in `nav2_behavior_tree`
- `nav2_behaviors::Spin` is the behavior plugin loaded by `behavior_server`
- the former sends a ROS action request to the latter
