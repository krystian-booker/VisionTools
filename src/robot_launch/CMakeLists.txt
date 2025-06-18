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
# via ament_index_python.packages.get_package_share_directory.
# CMAKE_SOURCE_DIR refers to the top-level directory of the project,
# which in a colcon workspace is the root of your repository.
install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()