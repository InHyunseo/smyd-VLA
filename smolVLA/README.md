# smolVLA

[SmolVLA](https://huggingface.co/HuggingFaceVLA/smolvla_libero)를 [LIBERO](https://libero-project.github.io/) 과제로 CPU에서 돌려 보는 ROS2 프로젝트입니다.
설치와 실행 절차는 **[MANUAL.md](MANUAL.md)** 에 있습니다.

## 무엇을 하나

- **Open-loop (ROS2):** 녹화된 시연을 재생하며 모델이 예측한 action을 녹화된 action과 비교합니다. RViz2에 카메라 화면과 손끝 궤적(실제 vs 예측)을 띄우고, bag으로 기록해 그림과 영상을 만듭니다.
- **Closed-loop (시뮬레이터):** 모델이 자기 행동의 결과를 보며 과제를 수행합니다. ROS launch로 LeRobot 공식 평가를 시작하며, 성공률과 예측 궤적을 겹친 영상을 남깁니다.

```text
LIBERO 에피소드 (카메라 2대 + 손끝 상태 + 명령)
        │
        ▼
SmolVLA (CPU) ──▶ 50스텝 action chunk
        │
        ├── open-loop: 녹화된 action과 비교 (RViz2, 그림)
        └── closed-loop: 시뮬레이터에서 실행 (성공률, 영상)
```

| 구성 | 값 |
| --- | --- |
| 모델 | [`HuggingFaceVLA/smolvla_libero`](https://huggingface.co/HuggingFaceVLA/smolvla_libero) (LIBERO로 fine-tuning) |
| Open-loop 데이터 | [`HuggingFaceVLA/libero`](https://huggingface.co/datasets/HuggingFaceVLA/libero) |
| Closed-loop 환경 | LIBERO 시뮬레이터 (기본값 LIBERO-Object) |
| 실행 환경 | Ubuntu 22.04, ROS2 Humble, CPU only |

## 패키지

| 패키지 | 역할 |
| --- | --- |
| `smyd_vla_msgs` | 상태 벡터와 action chunk 메시지 |
| `smyd_vla_replay` | LIBERO 에피소드를 로봇·카메라 대신 재생하며 `/clock` 발행 |
| `smyd_vla_inference` | SmolVLA 추론 노드, open-loop 평가, closed-loop 평가(공식 `lerobot-eval` + 기록) |
| `smyd_vla_bringup` | `open_loop.launch.py`, `closed_loop.launch.py`, 파라미터, RViz2 설정 → [설명](src/smyd_vla_bringup/smyd_vla_bringup.md) |
| `smyd_vla_analysis` | open-loop 그림·RViz2 녹화 영상, closed-loop 오버레이 영상 |

## 결과

| 항목 | 값 |
| --- | --- |
| 모델 로딩 | 약 20 s |
| 추론 시간 | 약 3 s (chunk 1개 = 미래 50스텝) |
| Open-loop 손끝 오차 (action 단위, 2번 에피소드) | 스텝 1에서 0.106, 스텝 50에서 0.127 |
| Closed-loop 성공률 | LIBERO-Object 과제 0: 100% (1판) |

## 한계와 다음 단계

| 한계 | 다음 단계 |
| --- | --- |
| CPU 추론이 chunk당 약 3초라 실시간 제어가 안 됩니다. Open-loop는 시계를 멈춰 두고, closed-loop는 한 판에 5–15분 걸립니다 | GPU 실행, chunk 재사용(`steps_per_inference` 늘리기), 양자화 |
| Open-loop 데이터가 모델의 학습에 포함됐을 수 있어(학습 분할 비공개) 오차가 일반화 성능을 뜻하지 않습니다 | 직접 수집한 에피소드나 공개된 검증 분할로 재측정 |
| 공개된 첫 데이터 파일 제약으로 open-loop는 0–2번 에피소드(LIBERO-10)만 재생합니다. Closed-loop 기본값인 LIBERO-Object와 과제가 달라 두 결과를 직접 비교할 수 없습니다 | 나머지 데이터 파일을 받아 두 루프의 과제 정렬 |
| Closed-loop 성공률이 1판 기준이라 통계적 의미가 약합니다 | 과제·시드를 늘려 평균 성공률 측정 |
| 실제 로봇 없이 시뮬레이터만 씁니다 | SO-100 등 실물 팔에 같은 인터페이스 연결 |

## License

MIT
