cmake_minimum_required(VERSION 3.20)
project(multi_robot_control)

find_package(ament_cmake REQUIRED)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

include(./.r2r/r2r_cargo.cmake)



# put ros package dependencies here.
r2r_cargo(rcl                    # we need the c ros2 api
          rcl_action             # as of r2r 0.1.0, we also need the action api
          geometry_msgs
          sensor_msgs
          tf2_msgs
          nav_msgs
          rosgraph_msgs
          ros_agent_msgs)

# install binaries
if(WIN32)
  set(SUFFIX ".exe")
else()
  set(SUFFIX "")
endif()

# Define the list of programs to install
# From [[bin]] name in cargo.toml
set(PROGRAMS_LIST
  tf_topic_combiner
  ros_agent
  ros_agent_msg_printer
)

# use a loop to install each program
foreach(program ${PROGRAMS_LIST})
  install(PROGRAMS ${CMAKE_SOURCE_DIR}/target/colcon/${program}${SUFFIX}
          DESTINATION lib/${PROJECT_NAME})
endforeach()

# we need this for ros/colcon
ament_package()
