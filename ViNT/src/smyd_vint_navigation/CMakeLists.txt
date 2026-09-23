cmake_minimum_required(VERSION 3.8)
project(smyd_vint_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(onnxruntime_vendor REQUIRED)

add_library(vint_model src/vint_model.cpp)
target_include_directories(vint_model PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_link_libraries(vint_model PUBLIC onnxruntime::onnxruntime ${OpenCV_LIBS})

add_executable(vint_navigator_node src/vint_navigator_node.cpp)
ament_target_dependencies(vint_navigator_node
  rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge)
target_link_libraries(vint_navigator_node vint_model)

add_executable(waypoint_follower_node src/waypoint_follower_node.cpp)
ament_target_dependencies(waypoint_follower_node rclcpp geometry_msgs std_msgs)

install(TARGETS vint_navigator_node waypoint_follower_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
