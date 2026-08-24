cmake_minimum_required(VERSION 3.20)
project(rebuild_posital)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(canopen REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

cogen_dcf(posital)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Inclination.msg"
  DEPENDENCIES std_msgs
)

add_library(rebuild_posital_protocol SHARED src/protocol.cpp)
target_include_directories(rebuild_posital_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(rebuild_posital_hardware SHARED src/posital_hardware_interface.cpp)
target_include_directories(rebuild_posital_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rebuild_posital_hardware PUBLIC
  rebuild_posital_protocol
  canopen_core::device_container
  canopen_proxy_driver::proxy_driver
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
)
pluginlib_export_plugin_description_file(hardware_interface rebuild_posital_hardware_plugin.xml)

generate_parameter_library(pitch_broadcaster_parameters
  src/pitch_broadcaster.yaml
)

add_library(rebuild_posital_inclinometer_broadcaster SHARED src/pitch_broadcaster.cpp)
target_include_directories(rebuild_posital_inclinometer_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(rebuild_posital_inclinometer_broadcaster PUBLIC
  ${cpp_typesupport_target}
  controller_interface::controller_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  pitch_broadcaster_parameters
)
pluginlib_export_plugin_description_file(controller_interface posital_broadcaster_plugin.xml)


ament_add_gtest(
  test_load_posital_hardware_interface
  test/test_load_posital_hardware_interface.cpp
)

target_link_libraries(test_load_posital_hardware_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
)

install(
  TARGETS rebuild_posital_protocol rebuild_posital_hardware rebuild_posital_inclinometer_broadcaster pitch_broadcaster_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY
  launch
  description
  DESTINATION share/${PROJECT_NAME}
)
install(FILES config/posital_controllers.yaml DESTINATION share/${PROJECT_NAME}/config)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  canopen_core canopen_proxy_driver controller_interface hardware_interface pluginlib
  rclcpp rclcpp_lifecycle realtime_tools
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_flake8
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
