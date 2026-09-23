# ViNT 설치와 실행

프로젝트 설명은 [README.md](README.md)에 있습니다. 아래 명령은 `ViNT/` 폴더에서 실행합니다.

## 1. 준비와 빌드

Ubuntu 22.04, ROS 2 Humble Desktop, Gazebo Classic, `ros-dev-tools`, `python3-venv`가 필요합니다. Windows에서는 WSLg가 있는 WSL2를 사용할 수 있습니다. GPU는 필요하지 않지만 CPU 렌더링은 느릴 수 있습니다.

```bash
git clone https://github.com/inhyunseo/smyd-VLA.git
cd smyd-VLA/ViNT
sudo apt install -y python3-venv
./scripts/setup.sh
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

`setup.sh`는 `.venv/`와 공식 ViNT 코드·가중치를 준비한 뒤 `model/vint.onnx`를 만듭니다. 자동 다운로드가 실패하면 [ViNT 가중치 폴더](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg)의 `vint.pth`를 `cache/model_weights/vint.pth`에 두고 다시 실행합니다. 변환 결과는 `model/export_report.txt`에 기록됩니다. `rosdep`을 처음 쓴다면 `sudo rosdep init`을 한 번 실행합니다.

새 터미널에서 ROS 명령을 실행할 때마다 다음을 먼저 입력합니다. Python `.venv`는 모델 변환에만 쓰며 ROS 실행 중에는 활성화하지 않습니다.

```bash
cd smyd-VLA/ViNT
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 2. 시연 기록

기록 터미널에서 Gazebo를 시작합니다. `gui:=true`가 기본값이며 실제 화면에 Gazebo 창이 뜹니다.

```bash
ros2 launch smyd_vint_bringup record_topomap.launch.py name:=my_route
```

다른 터미널에서 로봇을 조종합니다.

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard
```

카메라가 나온 뒤 로봇을 천천히 이동시키고 다른 위치에서 멈춥니다. 기록 터미널에서 `Ctrl+C`로 종료합니다. `results/record_topomap/<시각>_my_route/`에는 순서대로 번호가 붙은 PNG, `poses.csv`, 원본 카메라·odom `bag/`이 저장됩니다. 화면 없이 실행하려면 `gui:=false`를 붙입니다. 이 경우에도 카메라 렌더링용 `DISPLAY`는 필요하며, Xvfb는 실제 화면이 없는 환경에서만 선택적으로 사용합니다.

## 3. 개루프 비교

기록 bag을 재생하고 ViNT의 경유점 예측을 기록된 이후 이동과 비교합니다.

```bash
ROUTE=$(ls -dt results/record_topomap/*_my_route | head -n 1)
ros2 launch smyd_vint_bringup open_loop.launch.py topomap:="$ROUTE"
OPEN_RUN=$(ls -dt results/open_loop/* | head -n 1)
ros2 run smyd_vint_analysis plot_run.py "$OPEN_RUN"
ros2 run smyd_vint_analysis render_video.py "$OPEN_RUN"
```

결과 폴더에 `waypoint_errors.csv`, `figure1.png`, `video.mp4`가 생깁니다. 그림은 로봇 기준 x·y 방향의 예측 오차와 전체 경유점 오차를 보여줍니다.

## 4. 폐루프 주행

기록 launch가 완전히 종료된 뒤 같은 topomap으로 자율주행합니다. 기록할 때와 같은 시작 위치에서 Gazebo가 새로 시작됩니다.

```bash
ros2 launch smyd_vint_bringup closed_loop.launch.py topomap:="$ROUTE"
CLOSED_RUN=$(ls -dt results/closed_loop/* | head -n 1)
ros2 run smyd_vint_analysis plot_run.py "$CLOSED_RUN"
ros2 run smyd_vint_analysis render_video.py "$CLOSED_RUN"
```

기록된 마지막 위치에서 0.5 m 이내에 도착하거나 300초가 지나면 종료됩니다. `Ctrl+C`로 중단해도 `result.yaml`과 `trajectory.csv`가 저장됩니다. 실행 폴더에는 원본 ROS `bag/`, `figure1.png`, `video.mp4`, `video_overlay.mp4`도 남습니다. `video.mp4`는 카메라 원본이고, `video_overlay.mp4`의 작은 좌표창은 카메라 화면에 투영한 경로가 아닌 로봇 기준 경유점입니다.

| 주요 토픽 | 용도 |
| --- | --- |
| `/camera/image_raw` | 시연 기록과 ViNT 입력 영상 |
| `/odom` | 기록·주행 위치 |
| `/scan` | 근접 장애물 이벤트 평가 |
| `/waypoint` | ViNT가 선택한 경유점 |
| `/cmd_vel` | 로봇 속도 명령 |
| `/vint/closest_node` | 선택된 topomap 노드 번호 |
| `/vint/inference_seconds` | ONNX 추론 시간 |
