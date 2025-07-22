cmake_minimum_required(VERSION 3.8)
project(robot_hardware)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Include headers
include_directories(
  include
)

# Plugin library
add_library(robot_hardware_plugin
  src/arduino_diff_drive_system.cpp
)

ament_target_dependencies(robot_hardware_plugin
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# Pluginlib export
pluginlib_export_plugin_description_file(hardware_interface diffbot_hardware_plugins.xml)

# Install targets
install(
  TARGETS robot_hardware_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install plugin XML
install(
  FILES diffbot_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Package exports
ament_export_include_directories(
  include
)

ament_export_libraries(
  robot_hardware_plugin
)

ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

ament_package()
