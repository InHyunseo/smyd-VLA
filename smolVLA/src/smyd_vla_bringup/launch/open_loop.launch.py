"""open_loop.launch.py
LIBERO 데이터셋 에피소드로 open-loop를 돌린다: 에피소드 재생, SmolVLA 추론, RViz2.
시계는 episode_replay_node가 /clock으로 내고, 나머지 노드는 use_sim_time으로 그 시계를 따른다.
오래 도는 프로세스 중 하나라도 끝나면(Ctrl+C, RViz2 창 닫기, 노드 오류) 전체를 종료한다.

출력: results/open_loop/<시각>_episode_NNNNNN/logs, bag (record:=true)
인자: dataset, episode (번호), task (설명 일부로 에피소드 찾기), loop, rviz, record
"""

from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.actions import SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

SIMULATION_TIME = {"use_sim_time": True}
RECORDED_TOPICS = [
    "/instruction", "/ground_truth/action", "/ground_truth/path",
    "/ground_truth/image", "/ground_truth/image2", "/prediction/chunk", "/prediction/path", "/tf",
]


def package_file(package, *path):
    """설치된 패키지 share 폴더 안의 파일 경로."""
    return PathJoinSubstitution([FindPackageShare(package), *path])


def launch_setup(context):
    """인자값으로 실행 폴더를 정하고 노드를 구성한다."""
    episode = int(LaunchConfiguration("episode").perform(context))
    run_directory = Path("results/open_loop").resolve() / f"{datetime.now():%Y%m%d_%H%M%S}_episode_{episode:06d}"
    run_directory.mkdir(parents=True)
    parameters_file = package_file("smyd_vla_bringup", "config", "open_loop.yaml")
    return [
        SetEnvironmentVariable("ROS_LOG_DIR", str(run_directory / "logs")),
        Node(
            package="smyd_vla_replay",
            executable="episode_replay_node.py",
            name="episode_replay_node",
            parameters=[parameters_file, {
                "dataset": LaunchConfiguration("dataset"),
                "episode": LaunchConfiguration("episode"),
                "task": LaunchConfiguration("task"),
                "loop": LaunchConfiguration("loop"),
            }],
            on_exit=Shutdown(),
        ),
        Node(
            package="smyd_vla_inference",
            executable="smolvla_node.py",
            name="smolvla_node",
            parameters=[parameters_file, SIMULATION_TIME],
            on_exit=Shutdown(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", package_file("smyd_vla_bringup", "rviz", "open_loop.rviz")],
            parameters=[SIMULATION_TIME],
            condition=IfCondition(LaunchConfiguration("rviz")),
            on_exit=Shutdown(),
        ),
        # sim time으로 기록한다. setsid는 Ctrl+C가 이 프로세스에 직접 전달되지 않게 한다.
        ExecuteProcess(
            cmd=["setsid", "ros2", "bag", "record", "--use-sim-time", "-o", str(run_directory / "bag"),
                 *RECORDED_TOPICS],
            condition=IfCondition(LaunchConfiguration("record")),
        ),
    ]


def generate_launch_description():
    """인자를 선언한다."""
    return LaunchDescription([
        DeclareLaunchArgument("dataset", default_value="HuggingFaceVLA/libero", description="Hugging Face repo id"),
        DeclareLaunchArgument("episode", default_value="0", description="에피소드 번호"),
        DeclareLaunchArgument("task", default_value="", description="과제 설명 일부로 에피소드 찾기"),
        DeclareLaunchArgument("loop", default_value="true", description="에피소드 반복 재생"),
        DeclareLaunchArgument("rviz", default_value="true", description="RViz2 실행"),
        DeclareLaunchArgument("record", default_value="false", description="실행 폴더에 bag 기록"),
        OpaqueFunction(function=launch_setup),
    ])
