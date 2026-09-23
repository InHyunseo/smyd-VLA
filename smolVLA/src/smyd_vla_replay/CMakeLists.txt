cmake_minimum_required(VERSION 3.8)
project(smyd_vla_replay)

find_package(ament_cmake REQUIRED)

install(PROGRAMS src/episode_replay_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()
