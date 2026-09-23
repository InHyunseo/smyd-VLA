# smyd_vint_experiment

시연을 기록하고 개루프·폐루프 결과를 파일로 남기는 노드들입니다.

- `topomap_recorder_node`: 주행 중 영상과 odom을 topomap으로 저장합니다.
- `open_loop_evaluator_node`: 예측 경유점을 이후 기록된 이동과 비교합니다.
- `closed_loop_evaluator_node`: 주행 궤적을 남기고 성공 여부를 판정합니다.

## 파일 규약

`topomap_recorder_node`는 `seconds_per_node`(기본 1.0초)마다 `NNNN.png`를 저장하고 `poses.csv`에 한 줄을 덧붙입니다.

| poses.csv 열 | 뜻 |
| --- | --- |
| `index` | 0부터 빠짐없이 증가하는 노드 번호. PNG 파일명과 같다 |
| `x`, `y`, `yaw` | 저장 시점의 odom 자세 |
| `seconds` | 영상의 시각 |

마지막 줄의 `x`, `y`가 폐루프의 목표 위치이고, 첫 줄의 자세가 폐루프 시작 위치입니다.

`open_loop_evaluator_node`의 `waypoint_errors.csv`는 예측 하나당 한 줄입니다. `predicted_*`는 발행된 경유점, `recorded_*`는 그 시점 로봇 기준으로 본 `(waypoint_index + 1) / model_frame_rate`초 뒤의 실제 이동, `error_m`은 둘 사이 거리입니다.

`closed_loop_evaluator_node`의 `trajectory.csv`는 odom 한 건마다 시각·위치·목표까지 거리를 적습니다. `result.yaml`은 종료 시 한 번 쓰며, 임시 파일에 쓴 뒤 이름을 바꾸므로 중간 상태가 남지 않습니다.

| result.yaml 항목 | 뜻 |
| --- | --- |
| `success` | 목표 위치 0.5 m 이내 도달 여부 |
| `termination_reason` | `success`, `model_stopped`(모델이 도착을 선언했지만 반경 밖), `timeout`, `interrupted` |
| `recorded_path_length_m` / `traveled_path_length_m` / `path_length_ratio` | 기록 경로 길이, 주행 길이, 그 비 |
| `final_distance_to_goal_m` | 종료 시점의 목표까지 거리 |
| `collision_events` | `/scan` 최솟값이 0.2 m 안으로 들어온 횟수. 실제 접촉 횟수가 아니다 |
| `model_claimed_goal` / `model_claimed_goal_distance_m` | 모델이 도착을 선언했는지와 그때의 실제 거리 |

## 판정 상수

성공 반경 0.5 m, 제한 시간 300초, 근접 임계 0.2 m는 `closed_loop_evaluator_node.cpp`에 상수로 있습니다. 성공 판정은 odom과 기록된 마지막 노드 위치 사이의 평면 거리이며, 모델의 도착 선언과는 무관합니다.

## Ctrl+C

세 노드 모두 중단해도 파일이 남습니다. `poses.csv`와 두 CSV는 줄마다 flush하고, `result.yaml`은 종료 경로에서 `interrupted`로 기록됩니다.
