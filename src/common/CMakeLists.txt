set(CMAKE_AUTOUIC ON)

add_library(${PROJECT_NAME} STATIC
   EulerWidget.cpp
   TransformWidget.cpp
   FramesWidget.cpp
   TransformBroadcaster.cpp
   ${UI_SOURCES}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES
   POSITION_INDEPENDENT_CODE ON
)
