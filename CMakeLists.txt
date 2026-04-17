cmake_minimum_required(VERSION 3.22)
project(agni_tf_tools)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(angles REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost CONFIG REQUIRED COMPONENTS program_options)

## QT stuff
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

add_subdirectory(src)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
