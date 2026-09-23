# ViNT 매뉴얼

설치부터 실행까지 순서대로 따라 하면 됩니다. 프로젝트 설명은 [README.md](README.md)에 있습니다.
1번 이후의 모든 명령은 저장소의 `ViNT/` 폴더에서 실행합니다.

## 0. 준비물

- Ubuntu 22.04 (Windows는 [WSL2](https://learn.microsoft.com/windows/wsl/install)로 설치, WSLg 포함)
- [ROS 2 Humble Desktop](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)과 `ros-dev-tools`
- Gazebo Classic (`ros-humble-turtlebot3-gazebo`가 함께 설치합니다)
- 디스크 여유 공간 5GB 이상. GPU는 필요 없지만 CPU 렌더링은 느립니다.

## 1. 저장소와 Python 환경

```bash
git clone https://github.com/InHyunseo/smyd-VLA.git
cd smyd-VLA/ViNT

sudo apt install -y python3-venv
./scripts/setup.sh
```

스크립트가 하는 일은 [setup.sh](scripts/setup.sh) 맨 위에 적혀 있습니다. 결과로 `model/vint.onnx`와 변환 기록 `model/export_report.txt`가 생깁니다. 가중치 자동 다운로드가 막히면 [ViNT 가중치 폴더](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg)에서 `vint.pth`를 받아 `cache/model_weights/vint.pth`에 두고 다시 실행합니다.

## 2. ROS 의존성과 빌드

`rosdep`을 처음 쓴다면 `sudo rosdep init`을 한 번 실행합니다.

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

## 3. 터미널 환경

ROS 명령을 쓰는 터미널마다 먼저 실행합니다. Python `.venv`는 모델 변환에만 쓰며 여기서는 활성화하지 않습니다.

```bash
cd smyd-VLA/ViNT
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

## 4. 시연 기록

기록을 시작합니다. Gazebo 창이 뜨며, 화면 없이 돌리려면 `gui:=false`를 붙입니다.

```bash
ros2 launch smyd_vint_bringup record_topomap.launch.py name:=my_route
```

다른 터미널에서 3번을 실행한 뒤 로봇을 조종합니다.

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

천천히 주행하고 다른 위치에서 멈춘 다음, 기록 터미널에서 `Ctrl+C`로 끝냅니다. 로봇이 멈춰 있는 동안에는 노드가 쌓이지 않으므로 시작 전 대기 시간은 신경 쓰지 않아도 됩니다.

결과는 `results/record_topomap/<시각>_my_route/`에 남습니다. 다음 단계에서 쓰도록 경로를 변수에 담아 둡니다.

```bash
ROUTE=$(ls -dt results/record_topomap/*_my_route | head -n 1)
```

## 5. Open-loop 비교

```bash
ros2 launch smyd_vint_bringup open_loop.launch.py topomap:="$ROUTE"

RUN=$(ls -dt results/open_loop/* | head -n 1)
ros2 run smyd_vint_analysis plot_run.py "$RUN"
ros2 run smyd_vint_analysis render_video.py "$RUN"
```

## 6. Closed-loop 주행

4번의 launch가 완전히 끝난 뒤 실행합니다. 새 터미널이면 `ROUTE`를 다시 지정합니다.

```bash
ROUTE=$(ls -dt results/record_topomap/*_my_route | head -n 1)
ros2 launch smyd_vint_bringup closed_loop.launch.py topomap:="$ROUTE"

RUN=$(ls -dt results/closed_loop/* | head -n 1)
ros2 run smyd_vint_analysis plot_run.py "$RUN"
ros2 run smyd_vint_analysis render_video.py "$RUN"
```

기록된 마지막 위치에서 0.5 m 이내에 도착하거나 300초가 지나면 끝납니다. `Ctrl+C`로 중단해도 결과 파일은 남습니다.

## 7. 결과 폴더

```text
results/
├── record_topomap/<시각>_<name>/   0000.png…, poses.csv, bag/, logs/
├── open_loop/<시각>_<topomap>/     waypoint_errors.csv, figure1.png, video.mp4, logs/
└── closed_loop/<시각>_<topomap>/   result.yaml, trajectory.csv, bag/,
                                    figure1.png, video.mp4, video_overlay.mp4, logs/
```

각 파일의 내용은 [smyd_vint_experiment](src/smyd_vint_experiment/smyd_vint_experiment.md)와 [smyd_vint_analysis](src/smyd_vint_analysis/smyd_vint_analysis.md)에 정리돼 있습니다.
