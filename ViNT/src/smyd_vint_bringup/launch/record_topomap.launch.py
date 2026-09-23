# record_topomap.launch.py
# Gazebo와 TurtleBot3를 띄우고, 사람이 teleop으로 모는 동안 영상·odom을 topomap과 bag으로 남긴다.
#
# 입력: name, gui 인자
# 출력: results/record_topomap/<시각>_<name>/ (NNNN.png, poses.csv, bag/, logs/)

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
START_POSE = ("-2.0", "-0.5", "0.0")      # turtlebot3_house의 기본 시작 위치 (x, y, yaw)


def launch_setup(context):
    """결과 폴더를 만들고 시뮬레이터와 기록 노드를 구성한다."""
    name = LaunchConfiguration("name").perform(context)
    run_directory = Path("results/record_topomap").resolve() / f"{datetime.now():%Y%m%d_%H%M%S}_{name}"
    run_directory.mkdir(parents=True)
    turtlebot3_directory = Path(get_package_share_directory("turtlebot3_gazebo"))
    bringup_directory = Path(get_package_share_directory("smyd_vint_bringup"))
    robot_file = bringup_directory / "models" / "smyd_vint_waffle_pi" / "model.sdf"
    world_file = turtlebot3_directory / "worlds" / "turtlebot3_house.world"
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
            arguments=["-entity", ROBOT_MODEL, "-timeout", "300",   # 집 월드는 처음 열 때 오래 걸린다
                       "-file", str(robot_file),
                       "-x", START_POSE[0], "-y", START_POSE[1], "-z", "0.01", "-Y", START_POSE[2]]),
        ExecuteProcess(
            cmd=["ros2", "bag", "record", "--use-sim-time", "-o", str(run_directory / "bag"),
                 "/camera/image_raw", "/odom"],
            output="screen"),
        Node(
            package="smyd_vint_experiment", executable="topomap_recorder_node", output="screen",
            parameters=[{"use_sim_time": True,
                         "output_directory": str(run_directory),
                         "seconds_per_node": 1.0,
                         "meters_per_node": 0.05}],
            on_exit=Shutdown()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("name", default_value="route", description="topomap 이름"),
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo 창을 띄울지"),
        OpaqueFunction(function=launch_setup),
    ])
