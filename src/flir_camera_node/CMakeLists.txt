cmake_minimum_required(VERSION 3.5)
project(flir_camera_node)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    image_transport
    cv_bridge
    dynamic_reconfigure     
    message_generation
)

find_package(dynamic_reconfigure REQUIRED)  
generate_dynamic_reconfigure_options(
  cfg/Flir.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    image_transport
    cv_bridge
    dynamic_reconfigure     
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}    
  include
  /opt/spinnaker/include
  /opt/spinnaker/include/Spinnaker
)

link_directories(/opt/spinnaker/lib)

add_executable(${PROJECT_NAME}_node src/flir_camera_node.cpp)

add_dependencies(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_gencfg    
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Spinnaker
)
