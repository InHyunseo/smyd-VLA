cmake_minimum_required(VERSION 3.8)
project(smyd_vint_analysis)

find_package(ament_cmake REQUIRED)

install(PROGRAMS src/plot_run.py src/render_video.py DESTINATION lib/${PROJECT_NAME})

ament_package()
