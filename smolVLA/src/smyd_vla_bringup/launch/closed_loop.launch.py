"""closed_loop.launch.py
LIBERO 시뮬레이터에서 SmolVLA를 closed-loop로 평가한다. ROS 노드는 없고, evaluate_closed_loop.py
(LeRobot 공식 lerobot-eval 호출)를 실행한 뒤 끝나면 종료한다. CPU로 한 판에 약 5–15분 걸린다.

출력: results/closed_loop/<시각>_<suite>_task_NN/eval_info.json (성공률), videos/*.mp4
인자: suite (libero_spatial, libero_object, libero_goal, libero_10), task (과제 번호), episodes (판 수)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """인자를 선언하고 평가 스크립트를 실행한다."""
    return LaunchDescription([
        DeclareLaunchArgument("suite", default_value="libero_object", description="LIBERO suite"),
        DeclareLaunchArgument("task", default_value="0", description="과제 번호"),
        DeclareLaunchArgument("episodes", default_value="1", description="판 수"),
        ExecuteProcess(
            cmd=["ros2", "run", "smyd_vla_inference", "evaluate_closed_loop.py",
                 "--suite", LaunchConfiguration("suite"), "--task", LaunchConfiguration("task"),
                 "--episodes", LaunchConfiguration("episodes")],
            output="screen",
            on_exit=Shutdown(),
        ),
    ])
