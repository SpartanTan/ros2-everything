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

colcon build --symlink-install --packages-select my_robot_desc my_robot_controllers my_robot_hardware 