# smyd_vint_bringup

Gazebo 시뮬레이터와 노드들을 함께 띄우는 launch, 설정, 로봇 모델입니다.

| launch | 인자 | 결과 폴더 |
| --- | --- | --- |
| `record_topomap.launch.py` | `name` (기본 `route`), `gui` (기본 `true`) | `results/record_topomap/<시각>_<name>/` |
| `open_loop.launch.py` | `topomap` | `results/open_loop/<시각>_<topomap 이름>/` |
| `closed_loop.launch.py` | `topomap`, `gui` (기본 `true`) | `results/closed_loop/<시각>_<topomap 이름>/` |

세 launch 모두 `ROS_LOG_DIR`을 실행 폴더의 `logs/`로 돌리고, 주요 노드가 끝나면 전체를 종료합니다.

## 시뮬레이터

월드는 `turtlebot3_gazebo`의 `turtlebot3_house.world`이고 로봇은 `models/smyd_vint_waffle_pi/`입니다. 이 모델은 TurtleBot3 waffle_pi에서 카메라만 640×480 30 Hz에서 **160×120 4 Hz**로 바꾼 것입니다. 소프트웨어 렌더링에서 집 월드의 카메라가 따라오지 못해 줄였고, 4 Hz는 ViNT가 학습한 관측 주기입니다. 메시는 원본 `turtlebot3_common`을 그대로 참조합니다.

기록은 `(-2.0, -0.5, 0.0)`에서 시작합니다. 폐루프는 topomap `poses.csv` 첫 줄의 자세에서 시작하므로 기록과 주행의 odom 원점이 같습니다.

## 노드 구성

```text
gzserver (turtlebot3_house + smyd_vint_waffle_pi)
   │ camera/image_raw   odom   scan   clock
   ▼
vint_navigator_node ──▶ waypoint ──▶ waypoint_follower_node ──▶ cmd_vel
   │                                                              │
   └──▶ topoplan/reached_goal, vint/closest_node ────────────────┘
   │
   └──▶ closed_loop_evaluator_node ──▶ result.yaml, trajectory.csv
```

개루프는 시뮬레이터 대신 기록 bag을 재생하고, `closed_loop_evaluator_node` 자리에 `open_loop_evaluator_node`가 들어가며 `waypoint_follower_node`는 뜨지 않습니다.

## bag에 기록되는 토픽

| launch | 토픽 |
| --- | --- |
| `record_topomap` | `/camera/image_raw`, `/odom` |
| `closed_loop` | `/camera/image_raw`, `/odom`, `/waypoint`, `/cmd_vel`, `/vint/closest_node`, `/vint/inference_seconds` |

`open_loop`은 bag을 만들지 않고 기록 bag을 100배속으로 재생만 합니다. `/scan`은 평가에만 쓰고 기록하지 않습니다.

## 설정

`config/navigate.yaml`의 파라미터는 [smyd_vint_navigation](../smyd_vint_navigation/smyd_vint_navigation.md)에 정리돼 있습니다.
