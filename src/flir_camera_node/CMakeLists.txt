add_compile_options(-std=c++11)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /opt/spinnaker/include
  /opt/spinnaker/include/Spinnaker
)

link_directories(/opt/spinnaker/lib)

add_executable(${PROJECT_NAME}_node src/flir_camera_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Spinnaker
)
