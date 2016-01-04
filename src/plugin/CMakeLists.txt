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
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME} ${catkin_LIBRARIES})

set_target_properties(${PROJECT_NAME}_plugins
  PROPERTIES OUTPUT_NAME ${PROJECT_NAME}
)

# Install rules
install(TARGETS
  ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
