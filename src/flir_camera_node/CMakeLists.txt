cmake_minimum_required(VERSION 3.5)
project(flir_camera_node)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Threads REQUIRED)   # required by umich3
find_package(catkin REQUIRED
             COMPONENTS roscpp
                        message_generation
                        image_transport
                        cv_bridge)

catkin_package(INCLUDE_DIRS
               include
               LIBRARIES
               ${PROJECT_NAME}
               CATKIN_DEPENDS
               roscpp
               image_transport
               cv_bridge)

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
