cmake_minimum_required(VERSION 3.8)
project(robot_launch)

find_package(ament_cmake REQUIRED)

# Install the launch directory, making launch files available
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the config directory from the workspace root
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()