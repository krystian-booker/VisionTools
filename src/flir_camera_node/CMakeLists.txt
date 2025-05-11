cmake_minimum_required(VERSION 3.5)
project(flir_camera_node)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  dynamic_reconfigure
  sensor_msgs
  camera_info_manager
  message_generation
)

find_package(dynamic_reconfigure REQUIRED)
generate_dynamic_reconfigure_options(
  cfg/Flir.cfg
)

add_message_files(
  FILES
    CameraSettingsRange.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
    ${CMAKE_CURRENT_BINARY_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    image_transport
    cv_bridge
    dynamic_reconfigure
    sensor_msgs
    camera_info_manager
    message_runtime
)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  /opt/spinnaker/include
  /opt/spinnaker/include/Spinnaker
)

link_directories(/opt/spinnaker/lib)

add_executable(${PROJECT_NAME}
  src/flir_camera_node.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Spinnaker
)
