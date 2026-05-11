cmake_minimum_required(VERSION 3.8)
project(peak_ros_interfaces)

find_package(ament_cmake REQUIRED)

set(dependencies
  rosidl_default_generators
  std_msgs
)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/Ascan.msg"
  "msg/Observation.msg"
  "srv/SendCommand.srv"
  DEPENDENCIES
  std_msgs
)

ament_package()
