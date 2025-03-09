cmake_minimum_required(VERSION 3.20)
project(multi_robot_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# we need this for ros/colcon
ament_package()
