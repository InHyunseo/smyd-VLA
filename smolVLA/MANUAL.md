# smolVLA 매뉴얼

설치부터 실행까지 순서대로 따라 하면 됩니다. 프로젝트 설명은 [README.md](README.md)에 있습니다.
2번 이후의 모든 명령은 저장소의 `smolVLA/` 폴더에서 실행합니다.

## 0. 준비물

- Ubuntu 22.04 (Windows는 [WSL2](https://learn.microsoft.com/windows/wsl/install)로 설치)
- [ROS2 Humble Desktop](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)과 `ros-dev-tools`
- 디스크 여유 공간 10GB 이상 권장 (Python 환경, 모델, 데이터셋 캐시). GPU는 필요 없습니다.

## 1. VSCode

1. Windows에 [VSCode](https://code.visualstudio.com/)를 설치합니다.
2. 확장에서 **WSL**, **Python**을 설치합니다.
3. 2번에서 받은 저장소 루트에서 `code .`를 실행합니다. 좌측 하단에 `WSL: Ubuntu-22.04`가 보이면 됩니다. ([WSL에서 VSCode 쓰기](https://code.visualstudio.com/docs/remote/wsl))

Python 인터프리터와 import 경로는 저장소 루트의 `.vscode/settings.json`에 설정되어 있습니다.

## 2. 저장소와 Python 환경

```bash
git clone https://github.com/InHyunseo/smyd-VLA.git
cd smyd-VLA/smolVLA

sudo apt install -y python3-venv
./scripts/setup.sh            # 가상환경 생성 + Python 의존성 설치 (--model 을 붙이면 모델도 미리 받음)
```

스크립트가 하는 일은 [setup.sh](scripts/setup.sh) 맨 위에 적혀 있습니다. 설치 스크립트와 이 프로젝트의 closed-loop 평가는 LIBERO 설정을 기본적으로 `.venv/libero_config/`에 저장하므로 다른 가상환경의 `~/.libero` 설정에 영향받지 않습니다. 별도 설정이 필요하면 `LIBERO_CONFIG_PATH`를 지정할 수 있습니다. 모델과 데이터셋은 실행할 때 필요한 만큼 Hugging Face 캐시(`~/.cache/huggingface`)로 받아집니다.

## 3. ROS 의존성과 빌드

`rosdep`을 처음 쓴다면 `sudo rosdep init`을 한 번 먼저 실행합니다. ([rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html))

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y     # xvfb, ffmpeg 등 설치
rosdep check --from-paths src --ignore-src          # 확인
colcon build --symlink-install
```

`rosdep check`가 `All system dependencies have been satisfied`를 출력하면 됩니다.

## 4. 터미널 환경

새 터미널을 열 때마다 이 세 줄을 먼저 실행합니다. 아래 5번의 명령은 모두 이 상태를 전제로 합니다.

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
```

## 5. 실행

### 5.1 Open-loop (ROS2)

녹화된 시연을 재생하고, 모델이 예측한 action을 녹화된 action과 비교합니다.

```bash
ros2 run smyd_vla_inference evaluate_open_loop.py                             # ROS 없이 모델만 확인 (첫 실행은 다운로드로 몇 분)
ros2 launch smyd_vla_bringup open_loop.launch.py                              # RViz2로 보기
ros2 launch smyd_vla_bringup open_loop.launch.py episode:=1                   # 에피소드 번호로 고르기
ros2 launch smyd_vla_bringup open_loop.launch.py record:=true loop:=false     # 한 번 재생하며 기록
```

RViz2에서 노란 선은 녹화된 손끝 궤적, 파란 선은 모델이 예측한 앞으로의 궤적입니다. 모델은 매번 미래 50스텝을 예측하지만, 기본 설정에서는 녹화된 1프레임(0.1초)을 재생한 뒤 다시 추론합니다. CPU 추론은 한 번에 약 3초이므로 전체 에피소드 재생에는 시간이 걸립니다. 추론 동안 시뮬레이션 시계는 멈춥니다. 종료는 `Ctrl+C` 또는 RViz2 창 닫기입니다.

재생 스텝 수, 데이터셋 주기, 체크포인트는 `src/smyd_vla_bringup/config/open_loop.yaml`에서 바꿉니다.

**에피소드 선택.** 기본 데이터셋 [`HuggingFaceVLA/libero`](https://huggingface.co/datasets/HuggingFaceVLA/libero)에는 40개 과제의 1,693개 에피소드가 있지만, 공개된 첫 데이터 파일에 0–2번만 들어 있어 지금은 그 세 개만 재생할 수 있습니다. 없는 번호를 주면 replay 노드가 `episode N not available` 한 줄을 남기고 종료합니다.

**그림과 영상.** 기록한 실행 폴더를 넘기면 만들어집니다. 둘 다 첫 예측 시점부터 시작하며, 영상은 화면 없이 가상 화면에서 녹화됩니다. 그림 설명은 [plot_open_loop.py](src/smyd_vla_analysis/src/plot_open_loop.py)의 그림 함수에 있습니다.

```bash
RUN=$(ls -dt results/open_loop/*_episode_000000 | head -n 1)          # 가장 최근 0번 에피소드
ros2 run smyd_vla_analysis plot_open_loop.py "$RUN"                   # figure1.png + 성분별 오차
ros2 launch smyd_vla_analysis render_open_loop.launch.py run:="$RUN"  # video.mp4
```

### 5.2 Closed-loop (시뮬레이터)

모델이 시뮬레이터에서 자기 행동의 결과를 보며 과제를 수행합니다. ROS launch로 시작하지만 평가 자체는 ROS 노드 없이 LeRobot 공식 평가(`lerobot-eval`)를 실행하고, 결과만 이 저장소 규칙에 맞춰 저장합니다. CPU로 한 판에 약 5–15분 걸립니다.

```bash
ros2 launch smyd_vla_bringup closed_loop.launch.py                                    # libero_object 과제 0, 1판
ros2 launch smyd_vla_bringup closed_loop.launch.py suite:=libero_goal task:=3 episodes:=5
```

과제 번호는 [smyd_vla_bringup.md](src/smyd_vla_bringup/smyd_vla_bringup.md#closed-loop-launch-인자)에 있습니다. 성공률은 `eval_info.json`의 `overall.pc_success`에 적힙니다.

**오버레이 영상.** 원본 영상은 그대로 두고, 예측 궤적을 겹친 영상을 따로 만듭니다.

```bash
CLOSED_RUN=$(ls -dt results/closed_loop/*_libero_object_task_00 | head -n 1)
ros2 run smyd_vla_analysis overlay_closed_loop.py "$CLOSED_RUN"
```

## 6. 결과 폴더

실행할 때마다 시각이 붙은 폴더가 하나 생깁니다.

```text
results/
├── open_loop/<시각>_episode_NNNNNN/    logs/, bag/, figure1.png, video.mp4
└── closed_loop/<시각>_<suite>_task_NN/ eval_info.json, episode_K.npz,
                                        videos/<suite>_<과제번호>/eval_episode_K.mp4
```
