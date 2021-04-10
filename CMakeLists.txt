cmake_minimum_required(VERSION 3.1)
project(agni_tf_tools)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  tf2_geometry_msgs
  tf2_ros
)

message(STATUS "Using Qt ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS program_options)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES agni_tf_tools
CATKIN_DEPENDS roscpp rviz
#  DEPENDS system_lib
)

## QT stuff
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

add_subdirectory(src)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
