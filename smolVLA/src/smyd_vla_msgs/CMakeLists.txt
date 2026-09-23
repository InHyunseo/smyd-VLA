cmake_minimum_required(VERSION 3.8)
project(smyd_vla_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/StateVector.msg
  msg/ActionChunk.msg
  DEPENDENCIES std_msgs)

ament_package()
