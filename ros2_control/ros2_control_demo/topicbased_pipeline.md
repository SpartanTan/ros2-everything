# TopicBasedSystem pipeline

通常`CM`驱动`hardware_interface::read()`读取状态，写入接口，然后`joint_state_broadcaster`通过`ResourceManager`的`LoanedStateInterface`获得关节状态（`controller_interface`也是从这个接口获得数据， `controller`和`joint_state_broadcaster`是同级的），发布`\joint_states`，然后`robot_state_publisher`接收`\joint_states`后转为`\tf`发布

在使用`TopicBasedSystem` 的情况下，joint states是由某个节点自主发布的，类似`tilt/joint_states`, `lift/joint_states`，这些话题在urdf中被配置，因此当有消息发布时，`CM`在`read()`阶段通过话题读取数据到硬件接口，然后`joint_state_broadcaster`（如果启动了的话）通过接口获取关节数据，发布`\joint_states`，然后`robot_state_publisher`接收`\joint_states`后转为`\tf`发布


**`TF`**
```txt
(外部节点发布)
tilt/joint_states  ----------------------+
                                         |
                                         v
                                  robot_state_publisher_tilt
                                         |
                                         v
                                      /tf, /tf_static


sideshift/joint_states  ---------------+
                                       |
                                       v
                              robot_state_publisher_sideshift
                                       |
                                       v
                                   /tf, /tf_static


lift/joint_states  --------------------+
                                       |
                                       v
                              robot_state_publisher_lift
                                       |
                                       v
                                   /tf, /tf_static


tricycle/joint_states  ----------------+
                                       |
                                       v
                              robot_state_publisher_drive
                                       |
                                       v
                                   /tf, /tf_static
```
**`ros2_control`**

```txt
(外部节点发布各子机构状态)
tilt/joint_states      \
sideshift/joint_states  \
lift/joint_states        +-->  TopicBasedSystem (hardware plugin)
tricycle/joint_states    /      - read(): 订阅/接收这些 joint_states topics
                        /       - 将数据写入 ros2_control 的 state_interfaces
                       /
                      v
             [state_interfaces: joint/position, joint/velocity, ...]

                      |
                      | controller_manager update loop
                      v

          +------------------------+
          | joint_state_broadcaster|
          | 读取 state_interfaces   |
          | 发布 /joint_states     |
          +------------------------+
                      |
                      v
                 /joint_states   (可供“单一 RSP”使用；你目前未用这条)


(控制命令闭环方向)
                 /cmd_vel
                    |
                    v
        +---------------------------+
        | lola_tricycle_controller  |
        | 计算目标轮速/关节目标      |
        | 写入 command_interfaces   |
        +---------------------------+
                    |
                    | controller_manager update loop
                    v
     [command_interfaces: joint/velocity, ...]
                    |
                    v
          TopicBasedSystem (hardware plugin)
          - write(): 将 command_interfaces 发布到对应命令 topic
                    |
                    v
     tricycle/joint_command, lift/joint_command, ...
     (由外部执行机构/仿真/下位机消费)

```