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

- 通过`/robot_description` topic获取URDF
- 解析 URDF 中 `<ros2_control>` 字段，生成硬件资源描述 
- 解析 URDF 中 `<ros2_control>`，生成硬件资源描述`HardwareInfo`并用 pluginlib 加载硬件插件
- 读取 [`controllers.yaml`](#3-configcontrollersyaml) 到自身参数服务器（用于 spawner/load_controller 等流程）
- 对外提供 controller 管理服务（load/configure/activate 等）
- 按 update_rate 运行控制循环：read → update → write

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

spawner 的 arguments 中传入的是 **控制器实例名**（instance name）：
- `joint_state_broadcaster`
- `cmdvel_diffdrive_controller`

controller_manager 会根据 `controllers.yaml` 中该实例名对应的 `type` 字段去加载真实的插件类型（plugin type）。
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster: <<<<<----- 控制器实例名 controller instance name
      type: joint_state_broadcaster/JointStateBroadcaster <<<<<----- plugin lookup key

    cmdvel_diffdrive_controller:
      type: my_robot_controllers/CmdVelDiffDriveController
```
因此，controller instance name可以随意修改，只要保证`controllers.yaml`中的与`launch`文件中传入的arguments部分一致即可。  
而type所持有的字符串需要和`my_robot_controllers_plugins.xml`中`class name=`部分一致，跳转[这里](#cmdvel-controller-name)

### 2. URDF 中 `<ros2_control>`

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

#### 2.1 `<hardware>`
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
#### 2.2 `<joint>`
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

### 3. `config/controllers.yaml`
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    cmdvel_diffdrive_controller:
      type: my_robot_controllers/CmdVelDiffDriveController

cmdvel_diffdrive_controller:
  ros__parameters:
    left_wheel_joint: left_wheel_joint
    right_wheel_joint: right_wheel_joint
    wheel_radius: 0.10
    wheel_separation: 0.40
    cmd_timeout: 0.5

```




## 3. my_robot_hardware

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

## 4. my_robot_controllers

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


3. 硬件插件初始化：on_init() 与 info_ 的来源

硬件插件通常继承：

hardware_interface::SystemInterface

3.1 info_ 是谁持有的

info_ 是硬件对象实例内部的成员变量（由基类 HardwareComponentInterface 持有，通常为 protected）：

每个硬件实例对应一个 info_

生命周期与硬件对象一致

3.2 为什么要在子类中调用 SystemInterface::on_init(info)

典型写法：

hardware_interface::CallbackReturn CanSystem::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 之后可用 info_ 读取 URDF 中 <hardware><param> 注入的参数
  if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters["can_interface"];
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}


调用 SystemInterface::on_init(info) 的作用：

执行基类通用初始化逻辑

将 info 拷贝进基类的 info_（更底层实现中会看到 info_ = hardware_info; 或 info_ = params.hardware_info;）

解析接口描述（joint_state_interfaces_ / joint_command_interfaces_ 等），供框架后续校验与管理

如果不调用基类 on_init()：

info_ 可能为空/未初始化

通用解析不发生

后续接口注册或 controller 激活可能失败

4. 导出接口：export/on_export_* 与“字符串绑定 + 指针执行”

硬件需要把可被控制器访问的“状态量/控制量”导出为接口对象。

4.1 StateInterface：状态量导出

（Humble 旧接口）：

states.emplace_back("left_wheel_joint", "position", &left_pos_);


（Jazzy 新接口）：

return { std::make_shared<StateInterface>("left_wheel_joint", "position", &left_pos_), ... };


含义分两部分：

字符串键："left_wheel_joint" + "position"

数据地址：&left_pos_

字符串用于“匹配”，指针用于“执行”。

4.2 CommandInterface：控制量导出

同理：

cmds.emplace_back("left_wheel_joint", "velocity", &cmd_left_vel_);


这个 &cmd_left_vel_ 是关键：控制器最终写入的就是这个地址。

5. controllers.yaml：实例名（name）与插件类型（type）

典型：

controller_manager:
  ros__parameters:
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    cmdvel_diffdrive_controller:
      type: my_robot_controllers/CmdVelDiffDriveController


joint_state_broadcaster / cmdvel_diffdrive_controller：控制器实例名

type: xxx/yyy：pluginlib 插件类型标识符（决定加载哪个库/哪个类）

spawner arguments 传入的是实例名；controller_manager 根据实例名查到 type，再通过 pluginlib 创建对象。

6. 控制器参数：left_wheel_joint: left_wheel_joint 的含义

例：

cmdvel_diffdrive_controller:
  ros__parameters:
    left_wheel_joint: left_wheel_joint


左边 left_wheel_joint：ROS parameter 名

右边 left_wheel_joint：parameter 的字符串值（通常对应 URDF joint 名）

控制器代码中：

auto_declare<std::string>("left_wheel_joint", left_wheel_joint_);


含义是：

声明参数 left_wheel_joint

默认值为 left_wheel_joint_ 当前内容

若 yaml 提供了该参数，则 yaml 值覆盖默认值

之后一般在 on_configure() 中读取：

left_wheel_joint_ = get_node()->get_parameter("left_wheel_joint").as_string();

7. 运行时控制循环：read → update → write

ros2_control_node（controller_manager）按固定频率执行控制循环：

hardware.read(time, period)

controller.update(time, period)（所有 active controllers）

hardware.write(time, period)

7.1 关节反馈数据从哪里来（JointStateBroadcaster）

JointStateBroadcaster 并不读 CAN

它读取的是硬件导出的 StateInterface（指针指向 left_pos_ / left_vel_ ...）

这些变量的更新来自 CanSystem::read() 解析 CAN 报文

因此：

是的：真实系统中 /joint_states 的数值通常来自 CanSystem::read() 从 CAN 收到并解析的反馈帧

如果 read() 不更新变量，则 /joint_states 将保持不变（常为 0）

7.2 控制命令 cmd_left_vel_ 从哪里来

控制器在 update() 中计算出 omega_l/omega_r

通过 LoanedCommandInterface::set_value() 写入 CommandInterface 的指针

指针指向硬件对象内部的 cmd_left_vel_ / cmd_right_vel_

因此：

cmd_left_vel_ 的值来自控制器

硬件在 write() 中读取它并发送

8. CAN 收发应放在哪里
8.1 on_configure()：打开 CAN 通道并做轻量自检

建议做：

打开 SocketCAN / 厂商通道

设置非阻塞/超时

读一次心跳或发一次“读状态”请求确认通信

不建议在 on_init() 做 IO。

8.2 write() / send_can_velocity：发送控制命令帧

真实系统中 send_can_velocity() 内部就应该调用 CAN 驱动 API，例如：

SocketCAN：构造 struct can_frame，::write(fd, ...)

厂商 SDK：Transmit(...)

建议：

send_can_velocity 只负责“打包 + 发一帧”

write() 中决定发哪些帧、频率、限幅、安全策略

8.3 read()：接收反馈帧并更新 state 变量

真实系统中 read() 应：

非阻塞读取（避免卡住控制周期）

解析反馈帧更新 left_pos_ / left_vel_ 等

超时/错误计数与故障策略（必要时返回 ERROR 或进入安全模式）

9. 一次完整链路示例（从 /cmd_vel 到 CAN，再到 /joint_states）

外部节点发布 /cmd_vel

CmdVelDiffDriveController 订阅 /cmd_vel，在 update() 计算：

左轮角速度 omega_l

右轮角速度 omega_r

控制器通过 set_value() 写入：

*(&cmd_left_vel_) = omega_l

*(&cmd_right_vel_) = omega_r

同周期 CanSystem::write() 读取 cmd_left_vel_ / cmd_right_vel_：

打包为 CAN 帧并发送到总线

电机驱动回传编码器/速度状态帧

CanSystem::read() 接收并解析 CAN 反馈，更新：

left_pos_ / left_vel_ / right_pos_ / right_vel_

JointStateBroadcaster 读取 StateInterface 指向的这些变量并发布 /joint_states

10. 关键约束与常见错误

joint 名必须与 URDF 完全一致（字符串匹配失败会导致 controller 激活失败）

CanSystem::on_init() 必须调用 SystemInterface::on_init(...)（否则 info_ 未填充，接口解析缺失）

read() 不更新 state 变量 → /joint_states 不动

right_can_id key 拼写错误会导致读取失败（例如 rihgt_can_id）

11. 术语快速对照

URDF joint name：系统中关节的唯一名称源头

hardware_parameters：URDF <hardware><param> 注入到 info_.hardware_parameters

StateInterface / CommandInterface：字符串键 + 指针地址的接口对象

Controller instance name：spawner 使用的名字（yaml 顶层键）

Controller type：pluginlib 类型标识（决定加载哪个 C++ 类）

read/update/write：实时控制循环固定顺序