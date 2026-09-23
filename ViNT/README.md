# ViNT visual navigation

[ViNT](https://github.com/robodhruv/visualnav-transformer)로 Gazebo Classic의 TurtleBot3를 주행시키는 ROS 2 Humble 프로젝트입니다. 공식 배포 흐름인 시연 기록 → topomap 생성 → 경유점 예측·추종을 ROS2와 C++로 옮겼습니다. [설치·실행 절차](MANUAL.md)는 별도 문서에 있습니다.

## 구성

카메라 영상과 `/odom`을 함께 기록해 topomap과 ROS bag을 만듭니다. ViNT ONNX 모델은 현재 영상 1장과 과거 영상 5장, topomap 후보 영상을 입력받아 경유점을 예측합니다. 전처리는 ONNX 그래프에 포함됩니다. 추종 노드는 경유점을 `/cmd_vel`로 바꾸며 입력이 끊기면 정지합니다. 기록과 주행은 같은 160×120, 5 Hz 카메라를 사용합니다.

| 패키지 | 역할 |
| --- | --- |
| `smyd_vint_navigation` | ViNT 추론, topomap 노드 선택, 경유점 추종 |
| `smyd_vint_experiment` | topomap 기록, 개루프 비교, 폐루프 평가 |
| `smyd_vint_bringup` | Gazebo 모델·설정과 기록·개루프·폐루프 launch |
| `smyd_vint_analysis` | 결과 그림과 카메라 영상 생성 |

개루프는 기록 bag을 ROS2로 재생해 ViNT의 0.75초 후 이동 예측을 이후 기록된 `/odom` 이동과 비교합니다. 폐루프는 로봇이 예측 경유점으로 움직이고, 기록된 마지막 위치까지의 거리로 성공을 판정합니다. 개루프 비교는 공식 배포판에 없는 이 프로젝트의 평가이며, **같은 시연을 topomap과 평가에 함께 사용**하므로 독립 테스트 점수가 아닙니다.

## 결과물

실행별 `results/` 폴더에 ROS bag과 평가 CSV·YAML이 저장됩니다. 분석 명령을 실행하면 `figure1.png`와 `video.mp4`가 생성되고, 폐루프에는 경유점·범례를 얹은 `video_overlay.mp4`도 생성됩니다. 폐루프의 `video.mp4`는 카메라 원본입니다. 개루프 그림은 경유점 예측 오차를, 폐루프 그림은 기록·주행 경로와 목표 거리를 보여줍니다. 각 실행의 수치는 해당 폴더에 남으며, 짧은 직선 시연 한 번의 결과를 일반적인 성공률로 해석할 수는 없습니다.

## 한계

- ViNT의 학습 영상과 Gazebo 영상은 다릅니다. 굽은 경로·반복되는 풍경·다른 초기 위치에서의 성능은 아직 확인되지 않았습니다.
- `/odom`과 `/scan`을 이용한 평가는 시뮬레이터 기준입니다. `collision_events`는 가까운 장애물에 들어간 횟수이며 물리적 충돌 횟수가 아닙니다.
- CPU 소프트웨어 렌더링에서는 Gazebo GUI가 느릴 수 있습니다. 화면 없이 실행해도 카메라 렌더링을 위한 그래픽 디스플레이는 필요합니다.

## 라이선스

자체 코드는 저장소 루트의 MIT 라이선스를 따릅니다. 수정한 TurtleBot3 SDF에는 [Apache-2.0 고지](src/smyd_vint_bringup/models/smyd_vint_waffle_pi/LICENSE-APACHE-2.0)를 포함했습니다. 다운로드한 ViNT 코드·가중치와 외부 의존성에는 각각의 라이선스가 적용됩니다.
