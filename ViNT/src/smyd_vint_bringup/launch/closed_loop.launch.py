"""Follow a recorded topomap in the TurtleBot3 house world."""

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
START_POSE = ("-2.0", "-0.5", "0.0")


def launch_setup(context):
    """Resolve paths and create one simulator plus the two C++ driving nodes."""
    turtlebot3_directory = Path(get_package_share_directory("turtlebot3_gazebo"))
    bringup_directory = Path(get_package_share_directory("smyd_vint_bringup"))
    world_file = turtlebot3_directory / "worlds" / "turtlebot3_house.world"
    robot_file = bringup_directory / "models" / "smyd_vint_waffle_pi" / "model.sdf"
    config_file = bringup_directory / "config" / "navigate.yaml"
    model_file = Path("model/vint.onnx").resolve()
    topomap_directory = Path(LaunchConfiguration("topomap").perform(context)).expanduser().resolve()
    if not model_file.is_file():
        raise RuntimeError(f"ONNX model not found: {model_file}")
    if not topomap_directory.is_dir():
        raise RuntimeError(f"topomap not found: {topomap_directory}")
    if not world_file.is_file() or not robot_file.is_file():
        raise RuntimeError("Gazebo world or low-resolution robot SDF not found")
    run_directory = Path("results/closed_loop").resolve() / f"{datetime.now():%Y%m%d_%H%M%S}_{topomap_directory.name}"
    run_directory.mkdir(parents=True)
    (run_directory / "source_topomap.txt").write_text(str(topomap_directory) + "\n")

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
                       "-x", START_POSE[0], "-y", START_POSE[1], "-z", "0.01", "-Y", START_POSE[2]]),
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
            parameters=[str(config_file), {"use_sim_time": True,
                                           "topomap_directory": str(topomap_directory),
                                           "output_directory": str(run_directory)}],
            on_exit=Shutdown()),
        ExecuteProcess(
            cmd=["ros2", "bag", "record", "--use-sim-time", "-o", str(run_directory / "bag"),
                 "/camera/image_raw", "/odom", "/waypoint", "/cmd_vel", "/vint/closest_node"],
            output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("topomap", description="record_topomap output directory"),
        DeclareLaunchArgument("gui", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
