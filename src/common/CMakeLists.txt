set(CMAKE_AUTOUIC ON)

add_library(${PROJECT_NAME} STATIC
   EulerWidget.cpp
   TransformWidget.cpp
   FramesWidget.cpp
   TransformBroadcaster.cpp
   ${UI_SOURCES}
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Qt5::Widgets
    Qt5::Core
    Eigen3::Eigen
    rclcpp::rclcpp
    angles::angles
    tf2_ros::tf2_ros
)
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
