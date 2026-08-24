# ros2_control_demo

## snippets
### create packages
```bash
ros2 pkg create my_robot_hardware \
  --build-type ament_cmake \
  --dependencies \
    rclcpp \
    hardware_interface \
    pluginlib

ros2 pkg create my_robot_controllers \
  --build-type ament_cmake \
  --dependencies \
    rclcpp \
    controller_interface \
    hardware_interface \
    pluginlib \
    geometry_msgs

ros2 pkg create my_robot_desc \
  --build-type ament_cmake
```

### build
```bash
colcon build --symlink-install --packages-select my_robot_desc my_robot_controllers my_robot_hardware 
```

### test
```bash
ros2 launch my_robot_desc bringup.launch.py simulation:=true
ros2 launch my_robot_desc bringup.launch.py simulation:=false
```

- if using topic_based
  ```bash
  ros2 topic pub -r 30 /diffbot/joint_states sensor_msgs/msg/JointState "{name: ['left_wheel_joint','right_wheel_joint'], position: [0.5,0.2], velocity: [0.
  0,0.0], effort: [0.0,0.0]}"

  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

  ```

  verify commands and states
  ```bash
  ros2 topic echo /diffbot/joint_command 
  ros2 topic echo /joint_states
  ```
- if using CanSystem
  ```bash
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
  ```
  You shall see the terminal prints the simulated can messages sending out.