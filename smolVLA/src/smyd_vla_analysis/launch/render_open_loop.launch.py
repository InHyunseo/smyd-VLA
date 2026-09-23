"""render_open_loop.launch.py
기록한 bag을 가상 화면(Xvfb)의 RViz2에서 재생하며 ffmpeg로 녹화한다. bag은 sim time으로 기록되어 있어
추론으로 멈췄던 구간이 없고, 재생한 에피소드만 이어진 영상이 된다.
ffmpeg가 bag 길이만큼 녹화하고 끝나면
RViz2 → 나머지 순으로 종료해, 가상 화면이 먼저 꺼져 녹화가 깨지는 일이 없게 한다.

인자: run (open_loop.launch.py record:=true로 만든 results/open_loop/<시각>_episode_NNNNNN, 안에 bag/)
출력: 같은 실행 폴더의 video.mp4
"""

import signal
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, Shutdown, TimerAction
from launch.events.process import SignalProcess
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DISPLAY = ":99"
SCREEN_SIZE = "1280x720"  # rviz/render_open_loop.rviz의 창 크기와 같아야 한다.
RVIZ_VIEW_CROP = "1280:630:0:62"  # 메뉴·툴바(위 62 px)와 상태바(아래)를 잘라 낸 영역 (너비:높이:x:y)
RVIZ_STARTUP_SECONDS = 5.0  # 소프트웨어 렌더링으로 RViz2가 뜰 때까지 기다린다.


def package_file(package, *path):
    """설치된 패키지 share 폴더 안의 파일 경로."""
    return PathJoinSubstitution([FindPackageShare(package), *path])


def stop(process):
    """다른 프로세스가 끝났을 때 process에 SIGINT를 보내는 동작."""
    return EmitEvent(event=SignalProcess(signal_number=signal.SIGINT, process_matcher=matches_action(process)))


def launch_setup(context):
    """실행 폴더로 입출력 경로를 정하고 가상 화면, RViz2, 녹화, bag 재생을 구성한다."""
    run_directory = Path(LaunchConfiguration("run").perform(context)).resolve()
    bag = run_directory / "bag"
    video = run_directory / "video.mp4"
    with open(bag / "metadata.yaml") as file:
        duration = yaml.safe_load(file)["rosbag2_bagfile_information"]["duration"]["nanoseconds"] * 1e-9
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", package_file("smyd_vla_analysis", "rviz", "render_open_loop.rviz")],
        parameters=[{"use_sim_time": True}],
        additional_env={"DISPLAY": DISPLAY, "LIBGL_ALWAYS_SOFTWARE": "1"},
        on_exit=Shutdown(),
    )
    ffmpeg = ExecuteProcess(
        cmd=["ffmpeg", "-y", "-loglevel", "error",
             "-f", "x11grab", "-draw_mouse", "0", "-video_size", SCREEN_SIZE, "-framerate", "30",
             "-t", f"{duration:.2f}", "-i", DISPLAY,
             "-vf", f"crop={RVIZ_VIEW_CROP}", "-c:v", "libx264", "-pix_fmt", "yuv420p",
             "-movflags", "+faststart", str(video)],
        on_exit=stop(rviz),
    )
    bag_play = ExecuteProcess(cmd=["ros2", "bag", "play", "--clock", "100", str(bag)])
    return [
        ExecuteProcess(cmd=["Xvfb", DISPLAY, "-screen", "0", f"{SCREEN_SIZE}x24"]),
        rviz,
        TimerAction(period=RVIZ_STARTUP_SECONDS, actions=[ffmpeg, bag_play]),
    ]


def generate_launch_description():
    """run 인자를 선언한다."""
    return LaunchDescription([
        DeclareLaunchArgument("run", description="results/open_loop/<시각>_episode_NNNNNN"),
        OpaqueFunction(function=launch_setup),
    ])
