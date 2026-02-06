# ros2_control 全流程解析（从 Launch → URDF → Hardware → Controller → CAN）

本文系统性解释 **ros2_control 在真实机器人中的完整运行链路**。

目标：

- 彻底理解数据从 `/cmd_vel` 到 CAN 的路径
- 理解 `/joint_states` 从哪里来
- 理解 controller / hardware / URDF 的关系
- 搞清楚 `info_`、interfaces、参数、pluginlib 等机制

---

# 一、系统总体架构

典型工程结构：

```
my_robot_description
    ├── urdf
    ├── controllers.yaml
    └── launch

my_robot_hardware
    └── CanSystem (SystemInterface)

my_robot_controllers
    └── CmdVelDiffDriveController
```

核心控制链路：

```
cmd_vel
   ↓
Controller
   ↓
CommandInterface (指针写入)
   ↓
Hardware::write()
   ↓
CAN 总线
```

反馈链路：

```
CAN反馈
   ↓
Hardware::read()
   ↓
StateInterface
   ↓
JointStateBroadcaster
   ↓
/joint_states
```

---

# 二、Launch 启动了什么？

通常有三个 Node：

## 1️⃣ ros2_control_node（controller_manager）

负责：

- 读取 URDF
- 解析 `<ros2_control>`
- 加载 hardware plugin
- 管理 controllers
- 运行控制循环：

```
read → update → write
```

它是 **整个控制系统的大脑**。

---

## 2️⃣ spawner（joint_state_broadcaster）

作用：

加载并激活：

```
joint_state_broadcaster
```

这个 controller 会：

- 读取所有 state interfaces
- 发布：

```
/joint_states
```

⚠️ 注意：

> 它不会读取 CAN  
> 它只读取 **hardware 的内存**

---

## 3️⃣ spawner（你的 controller）

例如：

```
cmdvel_diffdrive_controller
```

作用：

- 订阅 `/cmd_vel`
- 计算左右轮速度
- 写入 CommandInterface

---

# 三、URDF 是“唯一真相源”

## joint 名的唯一权威来源

```xml
<joint name="left_wheel_joint">
```

以后所有地方：

- hardware
- controller
- yaml

都必须使用这个名字。

否则：

```
Interface not found
```

系统直接启动失败。

---

## `<ros2_control>` 的作用

例如：

```xml
<ros2_control>
  <hardware>
    <plugin>my_robot_hardware/CanSystem</plugin>

    <param name="can_interface">can0</param>
    <param name="left_can_id">0x201</param>
  </hardware>
</ros2_control>
```

这些 `<param>`：

✅ **不会进入 ROS parameter server**

而是进入：

```
HardwareInfo.hardware_parameters
```

例如：

```
info.hardware_parameters["can_interface"] == "can0"
```

---

# 四、Hardware 初始化：info_ 从哪里来？

你的代码：

```cpp
SystemInterface::on_init(info);
```

为什么必须调用？

因为：

更底层的实现会执行：

```cpp
info_ = hardware_info;
```

---

## info_ 是谁的？

它是：

```
HardwareComponentInterface 的 protected 成员
```

也就是说：

> 每个 hardware 对象内部都有一个 info_

生命周期 = hardware 对象。

---

## 如果不调用基类 on_init？

会发生：

- info_ 未赋值
- joints 未解析
- interfaces 未注册

结果：

```
controller activate 失败
```

---

# 五、Interfaces：字符串绑定 + 指针执行

例如：

```cpp
states.emplace_back(
    "left_wheel_joint",
    HW_IF_POSITION,
    &left_pos_);
```

本质包含两部分：

---

## ✔ 字符串绑定（启动阶段）

```
joint + interface
```

用于：

- controller_manager 匹配
- 校验 URDF

---

## ✔ 指针绑定（运行阶段）

```
&left_pos_
```

controller 或 broadcaster 最终访问的是：

```
double*
```

不是字符串。

---

# 六、controllers.yaml 的本质

例如：

```yaml
cmdvel_diffdrive_controller:
  ros__parameters:
    left_wheel_joint: left_wheel_joint
```

含义：

左边：

```
parameter 名
```

右边：

```
parameter 值
```

---

## auto_declare 的作用

```cpp
auto_declare<std::string>("left_wheel_joint", left_wheel_joint_);
```

意思是：

> 声明参数 + 提供默认值

如果 YAML 存在：

👉 YAML 覆盖默认值。

---

# 七、控制循环（极重要）

固定顺序：

```
hardware.read()
controllers.update()
hardware.write()
```

不能改变。

---

## read() 做什么？

真实机器人中：

✅ 从 CAN 读取反馈  
✅ 解析 encoder  
✅ 更新：

```cpp
left_pos_
left_vel_
```

⚠️ 不需要 return 数据  
直接写成员变量。

---

## JointStateBroadcaster 如何得到数据？

链路：

```
left_pos_
   ↑
StateInterface(double*)
   ↑
controller_manager
   ↑
JointStateBroadcaster
```

所以：

> `/joint_states` = hardware.read() 写入的内存

---

## write() 做什么？

controller 已经写入：

```
cmd_left_vel_
```

你只需要：

```cpp
send_can_velocity(cmd_left_vel_);
```

---

# 八、cmd_left_vel_ 从哪里来？

来自 controller：

```cpp
command_interfaces_[i].set_value(omega);
```

内部其实是：

```
*double_ptr = omega;
```

没有 topic  
没有 copy  
没有锁  

👉 **直接内存写入**

这就是 ros2_control 快的原因。

---

# 九、真实机器人建议

## ✔ CAN 打开位置

推荐：

```
on_configure()
```

不要在：

```
on_init()
```

做 IO。

---

## ✔ read() 必须：

- 非阻塞
- 做超时保护
- 做单位转换（ticks → rad）

---

## ✔ write() 必须：

- 限幅
- 检查 enable
- 防 runaway

---

# 十、极重要总结

## ros2_control 的本质：

> **共享内存实时控制框架**

特点：

- controller 不发 topic 给 hardware
- hardware 不发 topic 给 controller
- 一切通过：

```
double*
```

完成。

---

## 一句话理解整条链路

### 控制：

```
cmd_vel
   ↓
controller
   ↓
double*
   ↓
write()
   ↓
CAN
```

---

### 反馈：

```
CAN
   ↓
read()
   ↓
double*
   ↓
JointStateBroadcaster
   ↓
/joint_states
```

---

# 最后一句（强烈建议记住）

> **URDF 定义结构  
> hardware 提供数据  
> controller 计算控制  
> ros2_control 负责调度。**

