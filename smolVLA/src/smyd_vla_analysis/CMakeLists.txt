cmake_minimum_required(VERSION 3.8)
project(smyd_vla_analysis)

find_package(ament_cmake REQUIRED)

install(PROGRAMS src/plot_open_loop.py src/overlay_closed_loop.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
