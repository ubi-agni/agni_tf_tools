QT4_WRAP_UI(UI_SOURCES euler.ui transform.ui frames.ui)

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
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
