#!/usr/bin/env python3
"""evaluate_closed_loop
LIBERO 시뮬레이터에서 SmolVLA를 closed-loop로 평가한다(ROS 없음). LeRobot 공식 평가(lerobot_eval.eval_main)를
그대로 실행하고, 옆에서 스텝별 손끝 위치와 예측 chunk만 기록한다. 기록은 overlay_closed_loop.py가 쓴다.
CPU로 한 판에 약 5–15분 걸린다.

입력: --suite (libero_spatial, libero_object, libero_goal, libero_10), --task (과제 번호), --episodes (판 수), --model
출력: results/closed_loop/<시각>_<suite>_task_NN/
      eval_info.json (성공률), videos/<suite>_<task>/eval_episode_K.mp4 (raw 영상),
      episode_K.npz (end_effector_positions [T, 3] m, predicted_chunks [T, 50, 7] action, world_to_pixel [4, 4],
                     task_description)
"""

import argparse
import io
import os
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")  # 화면 없이 렌더링한다.


def main():
    """기록 코드를 공식 함수 두 곳에 덧붙이고 공식 평가를 실행한 뒤 기록을 저장한다."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite", default="libero_object")
    parser.add_argument("--task", type=int, default=0)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--model", default="HuggingFaceVLA/smolvla_libero")
    args = parser.parse_args()
    output = Path("results/closed_loop") / f"{datetime.now():%Y%m%d_%H%M%S}_{args.suite}_task_{args.task:02d}"

    sys.stdin, terminal = io.StringIO("N\n"), sys.stdin  # LIBERO 첫 import의 데이터 경로 질문에 기본값으로 답한다.
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.envs.libero import LiberoEnv
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
    from lerobot.scripts.lerobot_eval import eval_main
    from robosuite.utils.camera_utils import get_camera_transform_matrix
    sys.stdin = terminal

    _, postprocessor = make_pre_post_processors(
        PreTrainedConfig.from_pretrained(args.model), pretrained_path=args.model,
        preprocessor_overrides={"device_processor": {"device": "cpu"}})
    episodes = []  # 판마다 {"end_effector_positions": [...], "predicted_chunks": [...], "world_to_pixel": ...}

    official_reset, official_render = LiberoEnv.reset, LiberoEnv.render
    official_get_action_chunk = SmolVLAPolicy._get_action_chunk

    def reset(self, *reset_args, **reset_kwargs):
        """공식 reset 뒤에 새 판 기록을 시작한다."""
        result = official_reset(self, *reset_args, **reset_kwargs)
        size = self.observation_height, self.observation_width
        world_to_pixel = get_camera_transform_matrix(self._env.sim, "agentview", *size)
        episodes.append({"end_effector_positions": [], "predicted_chunks": [], "world_to_pixel": world_to_pixel,
                         "task_description": self.task_description})
        return result

    def render(self):
        """공식 렌더링(영상 한 프레임) 시점의 손끝 위치를 기록한다."""
        robot = self._env.robots[0]
        episodes[-1]["end_effector_positions"].append(np.array(self._env.sim.data.site_xpos[robot.eef_site_id]))
        return official_render(self)

    def get_action_chunk(self, batch, noise=None, **kwargs):
        """공식 추론 결과(정규화된 chunk)를 실제 action 단위로 바꿔 기록한다."""
        chunk = official_get_action_chunk(self, batch, noise, **kwargs)
        episodes[-1]["predicted_chunks"].append(postprocessor(chunk)[0].float().numpy())
        return chunk

    LiberoEnv.reset, LiberoEnv.render, SmolVLAPolicy._get_action_chunk = reset, render, get_action_chunk
    sys.argv = [
        "lerobot-eval", f"--policy.path={args.model}", "--policy.device=cpu",
        "--env.type=libero", f"--env.task={args.suite}", f"--env.task_ids=[{args.task}]",
        f"--eval.n_episodes={args.episodes}", "--eval.batch_size=1", f"--output_dir={output}",
    ]
    eval_main()

    # 판이 끝나면 LiberoEnv가 스스로 reset하므로, 추론이 없는 빈 기록은 버린다.
    recorded = [episode for episode in episodes if episode["predicted_chunks"]]
    for index, episode in enumerate(recorded):
        np.savez(output / f"episode_{index}.npz", **{key: np.asarray(value) for key, value in episode.items()})
    print(f"saved {len(recorded)} episode records to {output.resolve()}")


if __name__ == "__main__":
    main()
