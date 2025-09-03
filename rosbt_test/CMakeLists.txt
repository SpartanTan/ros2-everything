cmake_minimum_required(VERSION 3.16)
project(rosbt_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(rosbt_test src/main.cpp)

ament_target_dependencies(bt_action_demo
  rclcpp rclcpp_action behaviortree_cpp behaviortree_ros2 example_interfaces
)
target_link_libraries(bt_action_demo behaviortree_cpp::behaviortree_cpp)
