# ViNT visual navigation

[ViNT](https://github.com/robodhruv/visualnav-transformer)로 Gazebo Classic의 TurtleBot3를 주행시키는 ROS 2 Humble 프로젝트입니다. 추론까지 포함한 런타임 전체가 C++입니다. 설치와 실행 절차는 **[MANUAL.md](MANUAL.md)** 에 있습니다.

## 무엇을 하나

- **시연 기록:** 로봇을 한 번 몰고 다니며 카메라 영상과 `/odom`을 저장합니다. 일정 시간·거리마다 뽑은 영상들이 topological map이 됩니다.
- **Open-loop:** 기록한 bag을 재생하며 ViNT가 낸 경유점을 그 뒤 실제로 기록된 이동과 비교합니다. 로봇은 움직이지 않습니다.
- **Closed-loop:** 같은 자리에서 시작해 ViNT가 topomap을 되짚어 가고, 기록된 마지막 위치까지의 거리로 성공을 판정합니다.

```text
카메라 영상 6장 (현재 1 + 과거 5)  +  topomap 후보 영상 (현재 노드 ±radius)
        │
        ▼
ViNT ONNX (CPU) ──▶ 후보별 시간 거리 ──▶ 가장 가까운 노드 = 현재 위치
        │
        └────────▶ 경유점 5개 ──▶ 그중 3번째 ──▶ cmd_vel
```

전처리(크기 조정·정규화)는 ONNX 그래프 안에 있어 C++ 쪽은 픽셀을 복사만 합니다. 노드 선택과 속도 계산은 상류 `deployment/src`의 `navigate.py`, `pd_controller.py`와 같은 규칙입니다.

| 구성 | 값 |
| --- | --- |
| 모델 | ViNT (EfficientNet-B0 + transformer, 약 30M) → `model/vint.onnx` |
| 입력 | 160×120 RGB 6장 + 목표 영상, 4 Hz |
| 출력 | 시간 거리 1개, 경유점 5개 (dx, dy, cos, sin) |
| 시뮬레이터 | Gazebo Classic, `turtlebot3_house.world`, waffle_pi |
| 실행 환경 | Ubuntu 22.04, ROS 2 Humble, CPU only |

## 패키지

| 패키지 | 역할 |
| --- | --- |
| `smyd_vint_navigation` | ViNT 추론, topomap 노드 선택, 경유점 추종 → [설명](src/smyd_vint_navigation/smyd_vint_navigation.md) |
| `smyd_vint_experiment` | 시연 기록, 개루프 비교, 폐루프 평가 → [설명](src/smyd_vint_experiment/smyd_vint_experiment.md) |
| `smyd_vint_bringup` | Gazebo 모델·설정과 세 launch → [설명](src/smyd_vint_bringup/smyd_vint_bringup.md) |
| `smyd_vint_analysis` | 결과 그림과 영상 → [설명](src/smyd_vint_analysis/smyd_vint_analysis.md) |

## 결과

집 월드에서 완만한 S자 경로(25노드, 3.87 m)를 한 번 기록하고, 같은 topomap으로 개루프 1회·폐루프 3회를 돌린 결과입니다.

| 항목 | 값 |
| --- | --- |
| ONNX 변환 오차 (PyTorch 대비) | 시간 거리 8.6e-06, 경유점 1.6e-05 |
| 추론 시간 | 후보 5–6장 한 배치에 약 0.05 s (CPU 2스레드) |
| 개루프 경유점 오차 (55회 비교) | 평균 0.105 m, 중앙값 0.087 m, 최댓값 0.203 m |
| 폐루프 성공 | 3판 중 2판 (0.5 m 이내 도달) |

| 폐루프 | 결과 | 소요 [s] | 주행/기록 경로 | 도착 거리 [m] | 근접 이벤트 |
| --- | --- | --- | --- | --- | --- |
| 1판 | 성공 | 17.6 | 0.81 | 0.499 | 1 |
| 2판 | 모델이 정지 | 20.3 | 0.86 | 0.556 | 1 |
| 3판 | 성공 | 18.2 | 0.82 | 0.498 | 2 |

2판은 모델이 마지막 노드에 닿았다고 스스로 판단해 멈춘 경우이며, 그 지점이 성공 반경 밖이었습니다. 세 판 모두 기록 경로보다 짧게 잘라 달렸습니다.

개루프 오차는 topomap을 만든 것과 같은 시연을 재생해 얻은 값이라 독립 테스트 점수가 아닙니다. `results/`에는 기록한 시연 하나와 개루프·폐루프 각 한 판의 그림·영상·원자료를 남겨 두었습니다. 표의 나머지 두 판은 수치만 남겼습니다.

## 한계와 다음 단계

| 한계 | 다음 단계 |
| --- | --- |
| ViNT는 실사 주행 영상으로 학습했고 Gazebo 영상은 질감과 조명이 다릅니다. 주행 중 시간 거리가 1.6–13으로 흔들려 노드 인식이 안정적이지 않습니다 | Gazebo 영상으로 fine-tuning, 또는 실제 카메라로 같은 실험 |
| 시나리오가 집 월드의 완만한 S자 경로 하나뿐이고 시작 위치도 기록 시작점으로 고정입니다. 같은 구간을 요 ±2 rad로 급하게 도는 경로로 기록했을 때는 두 번 다 경로를 벗어났습니다 | 월드·경로·시작 위치를 바꿔 가며 반복, 굽은 정도별 성공률 측정 |
| 성공 판정이 `/odom` 기준이라 사실상 참값을 씁니다. 세 판 모두 경로를 그대로 따라가지 않고 S자 안쪽을 질러갔습니다 | 상태 추정을 넣고, 경로 추종 오차도 함께 보고 |
| 소프트웨어 렌더링 때문에 카메라를 160×120 4 Hz로 낮췄습니다. 학습 데이터보다 화각이 좁고 해상도가 낮습니다 | GPU 렌더링, 광각 카메라 SDF로 재측정 |
| `collision_events`는 `/scan` 최솟값이 0.2 m 안으로 들어온 횟수이며 실제 접촉 횟수가 아닙니다 | Gazebo 접촉 센서로 교체 |
| Gazebo Classic은 지원이 끝났습니다 | 노드가 보는 토픽은 4개뿐이라 launch 두 개만 바꾸면 Ignition으로 옮길 수 있습니다 |

## 라이선스

자체 코드는 저장소 루트의 MIT 라이선스를 따릅니다. 수정한 TurtleBot3 SDF는 Apache-2.0이며 [고지](src/smyd_vint_bringup/models/smyd_vint_waffle_pi/LICENSE-APACHE-2.0)를 함께 둡니다. 내려받는 ViNT 코드와 가중치(MIT), 그 밖의 의존성에는 각각의 라이선스가 적용됩니다.
