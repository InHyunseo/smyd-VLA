# open_loop.launch.py
# 기록한 bag을 재생하면서 ViNT의 경유점 예측을 그 뒤 기록된 이동과 비교한다. 로봇은 움직이지 않는다.
#
# 입력: topomap 인자, model/vint.onnx
# 출력: results/open_loop/<시각>_<topomap 이름>/ (waypoint_errors.csv, logs/)

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    """입력을 확인하고 추론 노드, 비교 노드, bag 재생을 구성한다."""
    topomap = Path(LaunchConfiguration("topomap").perform(context)).expanduser().resolve()
    bag = topomap / "bag"
    model = Path("model/vint.onnx").resolve()
    if not (topomap / "poses.csv").is_file() or not (bag / "metadata.yaml").is_file():
        raise RuntimeError(f"topomap must contain poses.csv and bag/: {topomap}")
    if not model.is_file():
        raise RuntimeError(f"ONNX model not found: {model}")

    config = Path(get_package_share_directory("smyd_vint_bringup")) / "config" / "navigate.yaml"
    run = Path("results/open_loop").resolve() / f"{datetime.now():%Y%m%d_%H%M%S}_{topomap.name}"
    run.mkdir(parents=True)
    (run / "source_bag.txt").write_text(str(bag) + "\n")
    (run / "source_topomap.txt").write_text(str(topomap) + "\n")
    return [
        SetEnvironmentVariable("ROS_LOG_DIR", str(run / "logs")),
        Node(
            package="smyd_vint_navigation", executable="vint_navigator_node", output="screen",
            parameters=[str(config), {"use_sim_time": True, "model_path": str(model),
                                      "topomap_directory": str(topomap)}],
            on_exit=Shutdown()),
        Node(
            package="smyd_vint_experiment", executable="open_loop_evaluator_node", output="screen",
            parameters=[str(config), {"use_sim_time": True, "output_directory": str(run)}],
            on_exit=Shutdown()),
        TimerAction(period=3.0, actions=[ExecuteProcess(
            cmd=["ros2", "bag", "play", "--clock", "100", str(bag)],
            output="screen", on_exit=Shutdown())]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("topomap", description="record_topomap 결과 폴더"),
        OpaqueFunction(function=launch_setup),
    ])
