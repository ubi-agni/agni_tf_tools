add_library(${PROJECT_NAME}_plugins MODULE
  TransformPublisherDisplay.cpp
  euler_property.cpp
  rotation_property.cpp
  plugin_init.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_plugins
  ${PROJECT_NAME}
  rviz_common::rviz_common
  rviz_default_plugins::rviz_default_plugins
)

set_target_properties(${PROJECT_NAME}_plugins
  PROPERTIES OUTPUT_NAME ${PROJECT_NAME}
)

# Install rules
install(TARGETS
  ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(FILES euler_property.h rotation_property.h
  DESTINATION include/${PROJECT_NAME}
)
