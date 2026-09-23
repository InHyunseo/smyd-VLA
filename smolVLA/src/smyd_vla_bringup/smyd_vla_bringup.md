# smyd_vla_bringup

실행용 launch와 설정을 담은 패키지입니다.

- `open_loop.launch.py`: LIBERO 에피소드 재생 + SmolVLA 추론 + RViz2 (아래 노드 구성). 파라미터 `config/open_loop.yaml`, RViz2 설정 `rviz/open_loop.rviz`
- `closed_loop.launch.py`: LIBERO 시뮬레이터 평가. ROS 노드 없이 `evaluate_closed_loop.py`를 실행

## Open-loop 노드 구성

```text
episode_replay_node ── 관측(카메라 2장, 손끝 상태), 명령 ──▶ smolvla_node
        ▲                                                        │
        └──────────────── prediction/chunk ◀─────────────────────┘
        │
        ▼ ground_truth/path, prediction/path (Marker), ground_truth/image, ground_truth/image2 (Image)
      RViz2
```

1. `episode_replay_node`가 시계를 멈추고 현재 프레임의 관측을 보냅니다.
2. `smolvla_node`가 추론(CPU 약 3초)하고 50스텝 chunk를 보냅니다.
3. 시계가 다시 흐르면서 녹화된 다음 1프레임을 재생합니다(`steps_per_inference` 기본값). 50스텝은 예측 길이이며, 모델의 예측은 로봇을 움직이지 않고 비교 대상으로만 쓰입니다(open-loop).
4. 재생이 끝나면 1로 돌아갑니다.

시계는 `episode_replay_node`가 `/clock`으로 내고 RViz2는 `use_sim_time`으로 따릅니다. 시계는 재생할 때만 흐르므로, 추론 대기 구간은 기록과 그림, 영상의 시간축에 남지 않습니다.

## Open-loop launch 인자

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `dataset` | `HuggingFaceVLA/libero` | Hugging Face repo id. 상태 규약이 모델과 같아야 함 |
| `episode` | `0` | 에피소드 번호. 기본 데이터셋은 0–2번만 재생 가능 |
| `task` | (없음) | 과제 설명 일부로 에피소드 찾기. 결과 폴더 이름은 `episode` 값을 따름 |
| `loop` | `true` | `false`면 에피소드를 한 번 재생하고 종료 |
| `rviz` | `true` | `false`면 RViz2 없이 실행 |
| `record` | `false` | `true`면 실행 폴더의 `bag`에 기록 |

## Open-loop 파라미터 (`config/open_loop.yaml`)

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `frames_per_second` | `10.0` | 데이터셋 주기 [Hz] |
| `steps_per_inference` | `1` | 예측 50스텝 중 재생할 스텝 수 |
| `model_id` | `HuggingFaceVLA/smolvla_libero` | 추론에 쓸 체크포인트 |
| `camera_keys` | `[observation.images.image, observation.images.image2]` | 데이터셋의 카메라 항목 이름 |

## Closed-loop launch 인자

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `suite` | `libero_object` | `libero_spatial`, `libero_object`, `libero_goal`, `libero_10` |
| `task` | `0` | 과제 번호 (0–9) |
| `episodes` | `1` | 판 수 |

`libero_object`의 과제는 모두 "물건을 집어 바구니에 넣기"입니다.

| 번호 | 물건 | 번호 | 물건 |
| --- | --- | --- | --- |
| 0 | alphabet soup | 5 | tomato sauce |
| 1 | cream cheese | 6 | butter |
| 2 | salad dressing | 7 | milk |
| 3 | bbq sauce | 8 | chocolate pudding |
| 4 | ketchup | 9 | orange juice |
