cmake_minimum_required(VERSION 3.8)
project(smyd_vint_experiment)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

add_executable(topomap_recorder_node src/topomap_recorder_node.cpp)
ament_target_dependencies(topomap_recorder_node
  rclcpp sensor_msgs nav_msgs cv_bridge tf2 tf2_geometry_msgs)
target_link_libraries(topomap_recorder_node ${OpenCV_LIBS})

add_executable(closed_loop_evaluator_node src/closed_loop_evaluator_node.cpp)
ament_target_dependencies(closed_loop_evaluator_node
  rclcpp nav_msgs sensor_msgs std_msgs)

install(TARGETS topomap_recorder_node closed_loop_evaluator_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(open_loop_evaluator_node src/open_loop_evaluator_node.cpp)
ament_target_dependencies(open_loop_evaluator_node rclcpp geometry_msgs nav_msgs)
install(TARGETS open_loop_evaluator_node DESTINATION lib/${PROJECT_NAME})

ament_package()
