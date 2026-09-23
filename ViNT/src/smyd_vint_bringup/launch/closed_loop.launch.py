# closed_loop.launch.py
# 기록할 때와 같은 자리에 로봇을 놓고, ViNT가 topomap을 따라 스스로 주행하게 한 뒤 결과를 남긴다.
#
# 입력: topomap, gui 인자, model/vint.onnx
# 출력: results/closed_loop/<시각>_<topomap 이름>/ (result.yaml, trajectory.csv, bag/, logs/)

import csv
import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_MODEL = "turtlebot3_waffle_pi"


def start_pose(topomap_directory):
    """topomap 첫 노드의 odom 자세를 스폰 위치로 돌려준다."""
    with (topomap_directory / "poses.csv").open(newline="") as stream:
        first = next(csv.DictReader(stream))
    return first["x"], first["y"], first["yaw"]


def launch_setup(context):
    """입력을 확인하고 시뮬레이터, 주행 노드, 평가 노드를 구성한다."""
    turtlebot3_directory = Path(get_package_share_directory("turtlebot3_gazebo"))
    bringup_directory = Path(get_package_share_directory("smyd_vint_bringup"))
    world_file = turtlebot3_directory / "worlds" / "turtlebot3_house.world"
    robot_file = bringup_directory / "models" / "smyd_vint_waffle_pi" / "model.sdf"
    config_file = bringup_directory / "config" / "navigate.yaml"
    model_file = Path("model/vint.onnx").resolve()
    topomap_directory = Path(LaunchConfiguration("topomap").perform(context)).expanduser().resolve()
    if not model_file.is_file():
        raise RuntimeError(f"ONNX model not found: {model_file}")
    if not (topomap_directory / "poses.csv").is_file():
        raise RuntimeError(f"topomap must contain poses.csv: {topomap_directory}")
    run_directory = Path("results/closed_loop").resolve() / f"{datetime.now():%Y%m%d_%H%M%S}_{topomap_directory.name}"
    run_directory.mkdir(parents=True)
    (run_directory / "source_topomap.txt").write_text(str(topomap_directory) + "\n")
    pose = start_pose(topomap_directory)
    robot_description = (turtlebot3_directory / "urdf" / f"{ROBOT_MODEL}.urdf").read_text()

    return [
        SetEnvironmentVariable("ROS_LOG_DIR", str(run_directory / "logs")),
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            os.pathsep.join([str(turtlebot3_directory / "models"), os.environ.get("GAZEBO_MODEL_PATH", "")])),
        ExecuteProcess(
            cmd=["gzserver", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so",
                 str(world_file)],
            on_exit=Shutdown()),
        ExecuteProcess(cmd=["gzclient"], condition=IfCondition(LaunchConfiguration("gui"))),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"use_sim_time": True, "robot_description": robot_description}]),
        Node(
            package="gazebo_ros", executable="spawn_entity.py",
            arguments=["-entity", ROBOT_MODEL, "-timeout", "300", "-file", str(robot_file),
                       "-x", pose[0], "-y", pose[1], "-z", "0.01", "-Y", pose[2]]),
        Node(
            package="smyd_vint_navigation", executable="vint_navigator_node", output="screen",
            parameters=[str(config_file), {"use_sim_time": True,
                                           "model_path": str(model_file),
                                           "topomap_directory": str(topomap_directory)}],
            on_exit=Shutdown()),
        Node(
            package="smyd_vint_navigation", executable="waypoint_follower_node", output="screen",
            parameters=[str(config_file), {"use_sim_time": True}],
            on_exit=Shutdown()),
        Node(
            package="smyd_vint_experiment", executable="closed_loop_evaluator_node", output="screen",
            parameters=[{"use_sim_time": True,
                         "topomap_directory": str(topomap_directory),
                         "output_directory": str(run_directory)}],
            on_exit=Shutdown()),
        ExecuteProcess(
            cmd=["ros2", "bag", "record", "--use-sim-time", "-o", str(run_directory / "bag"),
                 "/camera/image_raw", "/odom", "/waypoint", "/cmd_vel",
                 "/vint/closest_node", "/vint/inference_seconds"],
            output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("topomap", description="record_topomap 결과 폴더"),
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo 창을 띄울지"),
        OpaqueFunction(function=launch_setup),
    ])
