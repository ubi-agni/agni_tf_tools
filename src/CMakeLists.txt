# static_transform_publisher
add_executable(${PROJECT_NAME}_static_transform_publisher
  static_transform_broadcaster_program.cpp
)
target_link_libraries(${PROJECT_NAME}_static_transform_publisher
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  Eigen3::Eigen
  Boost::program_options
)
set_target_properties(${PROJECT_NAME}_static_transform_publisher
  PROPERTIES OUTPUT_NAME static_transform_publisher
)

## QT stuff ##
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct cmake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)

add_subdirectory(common)

# static_transform_publisher_gui
add_executable(${PROJECT_NAME}_static_transform_publisher_gui
  static_transform_publisher_gui.cpp
)
target_link_libraries(${PROJECT_NAME}_static_transform_publisher_gui
  ${PROJECT_NAME}
  Boost::program_options
  Qt${QT_VERSION_MAJOR}::Widgets
)
set_target_properties(${PROJECT_NAME}_static_transform_publisher_gui
  PROPERTIES OUTPUT_NAME static_transform_publisher_gui
)

# rviz plugin lib
add_subdirectory(plugin)

# Install rules
install(TARGETS
  ${PROJECT_NAME}_static_transform_publisher
  ${PROJECT_NAME}_static_transform_publisher_gui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
