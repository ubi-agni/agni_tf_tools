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
find_package(Eigen3 REQUIRED)
find_package(Boost CONFIG REQUIRED COMPONENTS program_options)

if(Qt6_FOUND)
  set(QT_VERSION_MAJOR 6)
elseif(Qt5_FOUND)
  set(QT_VERSION_MAJOR 5)
endif()

add_subdirectory(src)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
