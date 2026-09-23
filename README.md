# smyd-VLA

한 저장소에 로봇 학습 프로젝트 두 개를 둡니다. 각 폴더가 독립된 ROS2 워크스페이스입니다.

| 폴더 | 내용 |
| --- | --- |
| [smolVLA](smolVLA/) | SmolVLA를 LIBERO 과제로 돌립니다. open-loop는 ROS2로 예측과 녹화를 비교하고, closed-loop는 시뮬레이터에서 과제 수행을 평가합니다. |
| [ViNT](ViNT/) | 카메라 경로를 기록한 뒤 ViNT ONNX·C++ ROS2 노드로 개루프 비교와 Gazebo 폐루프 주행을 평가합니다. |

환경은 Ubuntu 22.04, ROS2 Humble, CPU 기준입니다. 각 폴더의 README가 프로젝트를 설명하고, MANUAL이 설치와 실행을 안내합니다.

## License

MIT

The MIT license applies to original code in this repository; third-party libraries, models, datasets, and downloaded ViNT code remain under their respective licenses.
