cmake_minimum_required(VERSION 3.8)
project(peak_ros)

add_compile_options(-std=c++17)
#set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wconversion -Wunused -Wunreachable-code")

# Required for compiled dependencies
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)
FetchContent_Declare(
  peak_micropulse
  GIT_REPOSITORY https://github.com/sam-xl/peak_micropulse_driver
  GIT_TAG        NXT03-668/fix-ltpa-lockup
)
FetchContent_MakeAvailable(peak_micropulse)

find_package(ament_cmake REQUIRED)

set(dependencies
  rclcpp
  rosidl_default_generators
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_msgs
  tf2_ros
  tf2_sensor_msgs
  ament_index_cpp
  rclcpp_components
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
  "srv/StreamData.srv"
  "srv/TakeSingleMeasurement.srv"
  "srv/SendCommand.srv"
  DEPENDENCIES
  std_msgs
)

###########
## Build ##
###########

include_directories(
  include
)

add_library(
  ${PROJECT_NAME}_lib
  src/peak_nodelet.cpp
  src/reconstruction_nodelet.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  ${dependencies}
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_lib
  PeakMicroPulseHandler
  "${cpp_typesupport_target}"
)

###############
# Peak Node #
###############

rclcpp_components_register_node(${PROJECT_NAME}_lib PLUGIN "peak::PeakNodelet" EXECUTABLE peak_node)

#######################
# Reconstruction Node #
#######################

rclcpp_components_register_node(${PROJECT_NAME}_lib PLUGIN "peak::ReconstructionNodelet" EXECUTABLE reconstruction_node)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_lib
  DESTINATION lib
)

install(TARGETS
  peak_node reconstruction_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch config mps
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
