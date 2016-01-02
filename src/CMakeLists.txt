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

# Install rules
install(TARGETS
  ${PROJECT_NAME}_static_transform_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
