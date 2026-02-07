# ros2_control 端到端流程说明（从 launch 加载 URDF 到 CAN 收发）

本文以如下三包结构为例（自定义 controller + 自定义 hardware）解释整个系统从启动到运行的完整链路：

- `my_robot_desc`：URDF / controllers.yaml / launch
- `my_robot_hardware`：`hardware_interface::SystemInterface` 插件（例如 `CanSystem`）
- `my_robot_controllers`：`controller_interface::ControllerInterface` 插件（例如 `CmdVelDiffDriveController`）

注意存放urdf的包名一定**不能包含**"robot_description"，似乎会和ros自己的`\robot_description`冲突。  
`my_robot_description`这种就会造成在launch时参数加载报错，从而无法加载任何controller！！
（以上这句可能并不完全成立，实际情况有可能是`controllers.yaml`这个文件的绝对路径不能存在"robot_description"字符串）

目标链路：

`/cmd_vel` → Controller（计算左右轮速度）→ ros2_control CommandInterface（指针写入）→ Hardware `write()`（打包 CAN 帧并发送）  
同时：Hardware `read()`（解析 CAN 反馈）→ StateInterface（指针读取）→ JointStateBroadcaster → `/joint_states`

---

## my_robot_desc

### 1. launch file
典型 launch 里有 3 个 Node：

1) `control_node`：启动 `controller_manager/ros2_control_node`（即 `controller_manager` 主节点）。
2) `robot_state_publisher`：启动 `robot_state_publisher/robot_state_publisher`。
3) `jsb_spawner`：启动 `controller_manager/spawner`，加载并激活 `joint_state_broadcaster`
4) `ctrl_spawner`：启动 `controller_manager/spawner`，加载并激活你的自定义控制器（例如 `cmdvel_diffdrive_controller`）

#### 1.1 control_node：`ros2_control_node` 的职责
它读取 [`controllers.yaml`](#3-configcontrollersyaml) 参数，并订阅 `~/robot_description`（通常需要 remap 到 `/robot_description`）以获得 URDF，解析其中 `<ros2_control>`，加载硬件插件并开启控制循环，同时提供 `/controller_manager/*` 系列服务接口供 `spawner`调用（load/configure/activate 等）。
```python
robot_controllers = PathJoinSubstitution(
    [FindPackageShare("my_robot_desc"), "config", "controllers.yaml"]
)

control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_controllers],
    output="screen",
)
```
`ros2_control_node` 是 controller_manager 的主节点，承担以下工作：

- 启动`/controller_manager`节点，对外提供 controller 管理服务（load/configure/activate 等）
- 读取 [`controllers.yaml`](#3-configcontrollersyaml) 中`controller_manager`命名空间的参数到自身参数服务器（用于 spawner/load_controller 等流程）
- 通过`/robot_description` topic获取URDF
- 解析 URDF 中 `<ros2_control>`，生成硬件资源描述`HardwareInfo`，并根据`<hardware>`所配置的插件，用 pluginlib 加载硬件插件(本工程中为CanSystem)，然后调用`on_init`读取硬件参数；之后进入`configure/activate`等lifecyclenode的操作
- 按 update_rate 运行控制循环：read → update → write

到这一步controllers的定义已经被加载在参数中，但是controllers实例还没被加载。

#### 1.2 `robot_state_publisher`
它通过 `robot_description` 参数获得 `URDF`，发布 `/tf` 与 `/tf_static`，并将 `URDF` 发布到 `/robot_description`（供其他节点使用，包括 `ros2_control_node`）。
```python
robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[robot_description],
    output="screen",
)
```


#### 1.3 spawner：`joint_state_broadcaster_spawner` 与 `cmdvel_controller_spawner`
spawner 是一个“客户端工具程序”，它通过调用 `/controller_manager` 提供的服务完成：

- `load_controller`
- `configure_controller`
- `activate_controller`

**`joint_state_broadcaster`**
从hardware interface读取joint的states然后发布到`/joint_states`话题。  
（注意和joint_state_publisher区分）
```python
joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"],
    output="screen",
)
```
spawner 的 arguments 中传入的是 **控制器实例名**（instance name）：
- `joint_state_broadcaster`

controller_manager 会根据 `controllers.yaml` 中该实例名"joint_state_broadcaster"对应的 `type` 字段去加载真实的插件类型（plugin type）。
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster: <<<<<----- 控制器实例名 controller instance name
      type: joint_state_broadcaster/JointStateBroadcaster <<<<<----- plugin lookup key
```
因此，`controller instance name`可以随意修改，只要保证`controllers.yaml`中的与`launch`文件中传入的`arguments`部分一致即可。  
而type所持有的字符串需要和`my_robot_controllers_plugins.xml`中`class name=`部分一致，跳转[这里](#cmdvel-controller-name)

`joint_state_broadcaster`负责广播hardware的states，发布`/joint_states`消息，`robot_state_publisher`会接收这些关节消息，然后转换为`\tf`后发布。


**`cmdvel_controller_spawner`**
```python
cmdvel_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "cmdvel_diffdrive_controller",
        "--controller-manager", "/controller_manager",
        "--param-file", robot_controllers,
    ],
    output="screen",
)
```
根据`ros2_control` jazzy 版本的[release_notes](https://control.ros.org/jazzy/doc/ros2_control/doc/release_notes.html#:~:text=The%20controllers%20will%20now%20set%20use_global_arguments%20from%20NodeOptions%20to%20false%2C%20to%20avoid%20getting%20influenced%20by%20global%20arguments%20(Issue%20%3A%20%231684)%20(%231694).%20From%20now%20on%2C%20in%20order%20to%20set%20the%20parameters%20to%20the%20controller%2C%20the%20%2D%2Dparam%2Dfile%20option%20from%20spawner%20should%20be%20used.) , `--pram-file`这个参数是必须要在启动controller时提供。

```txt
From now on, in order to set the parameters to the controller, the --param-file option from spawner should be used.
```
### 2. `config/controllers.yaml`

#### 2.2 controller_manager

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
```
#### 2.1 joint_state_broadcaster
```yaml
joint_state_broadcaster:
  ros__parameters:
    type: joint_state_broadcaster/JointStateBroadcaster
    use_local_topics: True
    joints:
      - left_wheel_joint 
      - right_wheel_joint
    interfaces:
      - position
      - velocity
```
通过launch文件中`--param-file`导入
```python
joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "joint_state_broadcaster",
        "--controller-manager", "/controller_manager",
        "--param-file", robot_controllers,
    ],
    output="screen",
)
```
`joints`字段下的内容必须和`urdf`中定义的joint name完全一致。  
`use_local_topics`设为True后发布的话题变为`\joint_state_broadcaster\joint_states`，否则默认为`joint_states`


#### 2.3 cmdvel_diffdrive_controller
自定义控制器的参数文件
```yaml
cmdvel_diffdrive_controller:
  ros__parameters:
    left_wheel_joint: left_wheel_joint
    right_wheel_joint: right_wheel_joint
    wheel_radius: 0.10
    wheel_separation: 0.40
    cmd_timeout: 0.5
```
```
参数名              参数值
↓                   ↓
left_wheel_joint : left_wheel_joint
```
参数名需要和`cmdvel_diffdrive_controller`的实现中所读取的变量名一致，即以下代码片段中的字符串部分。
```c++

controller_interface::CallbackReturn
CmdVelDiffDriveController::on_configure(const rclcpp_lifecycle::State &)
{
  left_wheel_joint_ = get_node()->get_parameter("left_wheel_joint").as_string();
  right_wheel_joint_ = get_node()->get_parameter("right_wheel_joint").as_string();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  wheel_separation_ = get_node()->get_parameter("wheel_separation").as_double();
  cmd_timeout_ = get_node()->get_parameter("cmd_timeout").as_double();

}
```
右侧的参数值需要和URDF/ros2_control中定义的joint name一致，例如
```xml
<joint name="left_wheel_joint">
  <command_interface name="velocity"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```
如果不一致，在controller active时报错
```txt
Could not find resource left_wheel_joint_typo/velocity
```

#### 2.4 threedofbot_position_controller
上一节的差速控制器是”语义型”的，必须明确表示左轮/右轮对应的关节名称，否则无法计算。  
但是，对于一些`Generic controller`，只要给出一列joints，
```yaml
threedofbot_position_controller:
  ros__parameters:
    type: forward_command_controller/ForwardCommandController
    joints:
      - threedofbot_joint1
      - threedofbot_joint2
      - threedofbot_joint3
    interface_name: position
```
控制器会读取参数并进行配置
```c++
for (const auto & joint : params_.joints)
{
  command_interface_types_.push_back(joint + "/" + params_.interface_name);
}

ForwardControllersBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}
```
注意`ForwardCommandController`对于joint只支持一个interface，但是ros2_control的设计是支持多个interface的。

### 3. URDF 中 `<ros2_control>`
一切定义以URDF中的为准。

URDF 中典型片段：
```xml
<ros2_control name="MyRobotSystem" type="system">
  <hardware>
    <plugin>my_robot_hardware/CanSystem</plugin>
    <param name="can_interface">can0</param>
    <param name="left_can_id">0x201</param>
    <param name="right_can_id">0x202</param>
  </hardware>

  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

#### 3.1 `<hardware>`
**使用自定义硬件接口**

`<plugin>`
```xml
<plugin>my_robot_hardware/CanSystem</plugin>
```
这里的`<plugin>my_robot_hardware/CanSystem</plugin>`必须和`my_robot_hardware`中导出的`my_robot_hardware_plugin.xml`文件中所定义的`<class name="my_robot_hardware/CanSystem"`这里完全一致。
`my_robot_hardware/CanSystem`就是这个插件的唯一名字（ID）
```xml
<?xml version="1.0"?>
<library path="my_robot_hardware">
  <class
    name="my_robot_hardware/CanSystem"
    type="my_robot_hardware::CanSystem"
    base_class_type="hardware_interface::SystemInterface">
    <description>CAN-based SystemInterface demo</description>
  </class>
</library>
```
`param>` 
```xml
<param name="can_interface">can0</param>
<param name="left_can_id">0x201</param>
<param name="right_can_id">0x202</param>
```
这些内容不会进入 ROS 参数服务器（不是普通 ROS parameter），它会被 ros2_control 解析后填入 `HardwareInfo.hardware_parameters`：

```c++
info.hardware_parameters["can_interface"] == "can0"
info.hardware_parameters["left_can_id"] == "0x201"
```

之后在`my_robot_hardware`的实现中
```c++
if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters["can_interface"];
}
if (info_.hardware_parameters.count("left_can_id")) {
    left_can_id_ = std::strtol(info_.hardware_parameters["left_can_id"].c_str(), nullptr, 0);
}
if (info_.hardware_parameters.count("right_can_id")) {
    right_can_id_ = std::strtol(info_.hardware_parameters["right_can_id"].c_str(), nullptr, 0);
}
```

**使用`topic_based_ros2_control`**
```xml
<hardware>
  <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
  <param name="joint_commands_topic">/diffbot/joint_command</param>
  <param name="joint_states_topic">/diffbot/joint_states</param>
  <param name="sum_wrapped_joint_states">true</param>
  <param name="trigger_joint_command_threshold">-1</param>
</hardware>
```
`trigger_joint_command_threshold`要注意下，如果没有话题
#### 3.2 `<joint>`
这里表达的是
- 这个系统有哪些joint
- 每个joint暴露哪些state/commmand interfaces
- 硬件参数(CAN id, 减速比，限幅等)
硬件层面的能力/资源清单
```xml
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
```

- 由control_manager形成资源清单HardwareInfo
- hardware_interface必须导出这些接口，否则资源不完整，controller等无法获取
- controller_manager用来校验/仲裁资源，当controller申请`left_wheel_joint/velocity`(command)时，必须存在且未被占用  


与hardware_interface对应部分[跳转](#hardware_interface-export)
与hardware_interface对应部分[跳转](#hardware_interface-export)


## my_robot_hardware

### cpp实现
- 接口导出 <a id="hardware_interface-export"></a> 
    ```c++
    std::vector<hardware_interface::StateInterface> CanSystem::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> states;
        states.emplace_back("left_wheel_joint", hardware_interface::HW_IF_POSITION, &left_pos_);
        states.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &left_vel_);
        states.emplace_back("right_wheel_joint", hardware_interface::HW_IF_POSITION, &right_pos_);
        states.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &right_vel_);
        return states;
    }
    std::vector<hardware_interface::CommandInterface> CanSystem::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> cmds;
        cmds.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &cmd_left_vel_);
        cmds.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &cmd_right_vel_);
        return cmds;
    }
    ```
    或者像`topic_based_ros2_control`中直接使用`info_`中的joints信息来导出
    ```c++
    std::vector<hardware_interface::StateInterface> TopicBasedSystem::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Joints' state interfaces
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            const auto& joint = info_.joints[i];
            for (const auto& interface : joint.state_interfaces)
            {
            // Add interface: if not in the standard list then use "other" interface list
            if (!getInterface(joint.name, interface.name, i, joint_states_, state_interfaces))
            {
                throw std::runtime_error("Interface is not found in the standard list.");
            }
            }
        }

        return state_interfaces;
    }
    
    bool TopicBasedSystem::getInterface(const std::string& name, const std::string& interface_name,
                                    const size_t vector_index, std::vector<std::vector<double>>& values,
                                    std::vector<HandleType>& interfaces)
    {
        auto it = std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface_name);
        if (it != standard_interfaces_.end())
        {
            auto j = static_cast<std::size_t>(std::distance(standard_interfaces_.begin(), it));
            interfaces.emplace_back(name, *it, &values[j][vector_index]);
            return true;
        }
        return false;
    }
    ```

### my_robot_hardware_plugins.xml
```xml
<?xml version="1.0"?>
<library path="my_robot_hardware">
  <class
    name="my_robot_hardware/CanSystem"
    type="my_robot_hardware::CanSystem"
    base_class_type="hardware_interface::SystemInterface">
    <description>CAN-based SystemInterface demo</description>
  </class>
</library>
```
### topic_based_ros2_control
用ROS topic模拟硬件的ros2_control Hardware Plugin
让ros2_topic可以
- 不直接操作电机驱动
- 不通过CAN/串口/EtherCAT
- 通过ROS topic和“外部系统”通信

```
controller_manager
    ↓
Controllers
    ↓
Command interfaces
    ↓
--------------------------------
topic_based_ros2_control   ← fake hardware
--------------------------------
    ↓ publish
    cmd_topic

外部driver / sim
    ↓ publish
    state_topic
--------------------------------
topic_based_ros2_control
--------------------------------
    ↓
State interfaces
    ↓
Controllers
```

这样并不是real-time，不适合高频控制。  
但是适合差速底盘/移动机器人/中低频控制（20~100Hz）/仿真

## my_robot_controllers

### my_robot_controllers.xml
插件导出时的命名规范为
```
<package_name>/<exported_name>
e.g.
joint_state_broadcaster/JointStateBroadcaster
diff_drive_controller/DiffDriveController
```
```xml
<?xml version="1.0"?>
<library path="my_robot_controllers">
  <class
    name="my_robot_controllers/CmdVelDiffDriveController"
    type="my_robot_controllers::CmdVelDiffDriveController"
    base_class_type="controller_interface::ControllerInterface">
    <description>CmdVel to wheel velocity controller (custom)</description>
  </class>
</library>
```
- `path`: 插件所在的共享库文件(.so)的名字  
    pluginlib最终去加载`libmy_robot_controllers.so`  
    需要和CMakeLists.txt中生成的库名对应
    ```Make
    add_library(my_robot_controllers SHARED
    src/cmdvel_diffdrive_controller.cpp
    )
    ```
- `<class ...>`: 声明这个库里导出了一个插件类
- `name`: `name="my_robot_controllers/CmdVelDiffDriveController"` <a id="cmdvel-controller-name"></a>   
    这是pluginlib的lookup key  
    与`controllers.xml`中`type: my_robot_controllers/CmdVelDiffDriveController`对应  
    conttroller_manager把type所对应的字符串当作lookup key去寻找插件
- `type`: `type="my_robot_controllers::CmdVelDiffDriveController"` 
    真实C++类的全限定名
    pluginlib根据lookup key找到对应的条目，用type去实例化创建  
    controller类实现必须叫这个  
    `namespace my_robot_controllers { class CmdVelDiffDriveController ... }`  
    即与.cpp中最后EXPORT部分保持一致
    ```c++
    PLUGINLIB_EXPORT_CLASS(my_robot_controllers::CmdVelDiffDriveController,
                       controller_interface::ControllerInterface)
    ```
- `base_class_type`: `base_class_type="controller_interface::ControllerInterface">`  
    基类接口类型，pluginlib用来做类型约束
    与.cpp中最后EXPORT部分保持一致
    ```c++
    PLUGINLIB_EXPORT_CLASS(my_robot_controllers::CmdVelDiffDriveController,
                       controller_interface::ControllerInterface)
    ```

---------
