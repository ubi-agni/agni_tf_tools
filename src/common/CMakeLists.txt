if(CMAKE_VERSION VERSION_GREATER "3.0.2")
  # this is only supported since 3.0.2
  set(CMAKE_AUTOUIC ON)
else()
  set(UI_FILES euler.ui transform.ui frames.ui)
  if(rviz_QT_VERSION VERSION_LESS "5")
    QT4_WRAP_UI(UI_SOURCES ${UI_FILES})
  else()
    QT5_WRAP_UI(UI_SOURCES ${UI_FILES})
  endif()
endif()

## Declare a C++ executable
add_library(${PROJECT_NAME} STATIC
   EulerWidget.cpp
   TransformWidget.cpp
   FramesWidget.cpp
   TransformBroadcaster.cpp
   ${UI_SOURCES}
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES
   POSITION_INDEPENDENT_CODE ON
)
