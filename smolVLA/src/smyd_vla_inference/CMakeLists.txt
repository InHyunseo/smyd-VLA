cmake_minimum_required(VERSION 3.8)
project(smyd_vla_inference)

find_package(ament_cmake REQUIRED)

# 실행 스크립트와 공용 모듈(smolvla_policy.py)을 같은 폴더에 설치해 import되게 한다.
install(PROGRAMS
  src/smolvla_node.py src/evaluate_open_loop.py src/evaluate_closed_loop.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES src/smolvla_policy.py DESTINATION lib/${PROJECT_NAME})

ament_package()
