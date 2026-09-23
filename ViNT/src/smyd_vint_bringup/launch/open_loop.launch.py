"""Replay a recorded route and compare ViNT waypoints with subsequent odometry."""

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
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
            parameters=[{"use_sim_time": True, "output_directory": str(run)}],
            on_exit=Shutdown()),
        TimerAction(period=3.0, actions=[ExecuteProcess(
            cmd=["ros2", "bag", "play", "--clock", "100", str(bag)],
            output="screen", on_exit=Shutdown())]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("topomap", description="record_topomap output directory"),
        OpaqueFunction(function=launch_setup),
    ])
