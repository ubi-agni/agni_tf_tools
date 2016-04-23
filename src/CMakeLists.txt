# static_transform_publisher
add_executable(${PROJECT_NAME}_static_transform_publisher
  static_transform_broadcaster_program.cpp
)
target_link_libraries(${PROJECT_NAME}_static_transform_publisher
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
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
include_directories(common)

# static_transform_publisher_gui
add_executable(${PROJECT_NAME}_static_transform_publisher_gui
  static_transform_publisher_gui.cpp
)
target_link_libraries(${PROJECT_NAME}_static_transform_publisher_gui
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}
  ${QT_LIBRARIES}
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
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
