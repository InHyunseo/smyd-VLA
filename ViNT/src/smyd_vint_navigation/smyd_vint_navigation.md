# smyd_vint_navigation

ViNT 추론과 경유점 추종을 담당하는 패키지입니다. 파라미터는 `smyd_vint_bringup`의 `config/navigate.yaml`에서 옵니다.

- `vint_navigator_node`: 카메라 영상 6장과 topomap 후보 영상을 ONNX 모델에 넣어 현재 노드를 고르고 경유점을 발행합니다.
- `waypoint_follower_node`: 경유점을 `cmd_vel`로 바꿉니다.

## 노드 구성

```text
camera/image_raw ──▶ vint_navigator_node ──▶ waypoint (PoseStamped, base_link)
                            │                       │
                            │                       ▼
                            │              waypoint_follower_node ──▶ cmd_vel
                            ▼
      topoplan/reached_goal, vint/closest_node, vint/inference_seconds
```

`vint_navigator_node`는 현재 노드 주변 `[closest - radius, closest + radius + 1]` 구간의 영상을 한 번에 추론해, 시간 거리가 가장 짧은 노드를 현재 위치로 봅니다. 그 거리가 `close_threshold` 이하이면 다음 노드로 넘어갑니다. 마지막 노드에 닿으면 `topoplan/reached_goal`에 `true`를 냅니다. 이 규칙은 상류 [`deployment/src/navigate.py`](https://github.com/robodhruv/visualnav-transformer/blob/main/deployment/src/navigate.py)와 같습니다.

## 파라미터

| 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `max_v` | `0.2` | 최대 선속도 [m/s]. 경유점을 미터로 바꿀 때도 쓴다 |
| `model_frame_rate` | `4.0` | 경유점 사이 시간 간격의 역수 [Hz]. 카메라 주기와 같아야 한다 |
| `waypoint_index` | `2` | 예측 5개 중 따라갈 번호 |
| `radius` | `2` | 현재 노드 앞뒤 후보 범위. 상류 기본값은 4이며 CPU 추론 비용 때문에 낮췄다 |
| `close_threshold` | `3.0` | 이 시간 거리 이하이면 다음 노드로 넘어간다 |
| `inference_rate_hz` | `2.0` | 추론 주기 |
| `image_timeout_seconds` | `2.0` | 이 시간 동안 영상이 없으면 추론을 멈춘다 |
| `inference_threads` | `2` | ONNX Runtime 스레드 수 |
| `max_w` | `0.4` | 최대 각속도 [rad/s] |
| `control_rate_hz` | `10.0` | `cmd_vel` 발행 주기 |
| `waypoint_timeout_seconds` | `1.0` | 이 시간 동안 경유점이 없으면 정지한다 |

`max_v`, `model_frame_rate`, `waypoint_index`는 두 노드가 같은 값을 써야 해서 `/**:` 블록에 있습니다.

## 속도 계산

경유점 `(dx, dy)`는 이미 `max_v / model_frame_rate` 배로 미터가 된 값입니다. 추종 노드는 상류 [`pd_controller.py`](https://github.com/robodhruv/visualnav-transformer/blob/main/deployment/src/pd_controller.py)와 같이 계산합니다.

```
v = dx * model_frame_rate            (0 ~ max_v로 제한)
w = atan(dy / dx) * model_frame_rate (±max_w로 제한)
```

`dx`가 0에 가까우면 회전만 합니다. 후진은 하지 않습니다.

## 모델

`model/vint.onnx`는 `scripts/export_vint_onnx.py`가 만듭니다. 영상 크기 조정과 정규화가 그래프 안에 있어 `vint_model`은 픽셀을 복사만 합니다. 영상 크기, 관측 장수, 경유점 개수는 ONNX 입출력 shape에서 읽고 맞지 않으면 노드가 뜨지 않습니다.
