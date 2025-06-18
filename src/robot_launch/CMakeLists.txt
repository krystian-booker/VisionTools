cmake_minimum_required(VERSION 3.8)
project(robot_launch)

find_package(ament_cmake REQUIRED)

# Install the launch directory, making launch files available
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the config directory from the repo root
# This makes the .yaml files available to the launch file
install(
  DIRECTORY ../../config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
