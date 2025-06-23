cmake_minimum_required(VERSION 3.8)
project(robot_launch)

find_package(ament_cmake REQUIRED)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install the launch directory, making launch files available
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the config directory from the workspace root
# Note: The original path might cause issues if this package is ever used outside this specific workspace structure.
# For this repo, it's functional.
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()