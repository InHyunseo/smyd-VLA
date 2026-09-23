#!/usr/bin/env python3
"""evaluate_open_loop
ROS 없이 SmolVLA만 CPU로 돌려, LIBERO 데이터셋 에피소드 하나에서 추론 시간과 오차(open-loop)를 잰다.

입력: --model, --dataset, --episode, --runs
출력: 시작 프레임별 chunk shape와 추론 시간, 첫 실행을 뺀 평균 추론 시간,
      예측 chunk와 녹화된 action 사이의 성분별 평균 절대 오차 (action 단위, [-1, 1] 정규화)
"""

import argparse
import time

import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from smolvla_policy import SmolVLAPolicyRunner

OBSERVATION_KEYS = ["observation.images.image", "observation.images.image2", "observation.state", "task"]
ACTION_NAMES = ["x", "y", "z", "roll", "pitch", "yaw", "gripper"]


def main():
    """에피소드에서 고르게 뽑은 시작 프레임마다 chunk를 예측해 녹화된 action과 비교한다."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="HuggingFaceVLA/smolvla_libero")
    parser.add_argument("--dataset", default="HuggingFaceVLA/libero")
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--runs", type=int, default=5)
    args = parser.parse_args()

    dataset = LeRobotDataset(args.dataset, episodes=[args.episode])
    runner = SmolVLAPolicyRunner(args.model)
    last_start_frame = len(dataset) - runner.chunk_size
    if last_start_frame < 0:
        raise SystemExit(f"에피소드가 chunk({runner.chunk_size} 스텝)보다 짧습니다")
    start_frames = range(0, last_start_frame + 1, max(1, last_start_frame // args.runs))[: args.runs]

    latencies, errors = [], []
    for start_frame in start_frames:
        item = dataset[start_frame]
        start_time = time.perf_counter()
        predicted_chunk = runner.predict_chunk({key: item[key] for key in OBSERVATION_KEYS})
        latencies.append(time.perf_counter() - start_time)

        recorded_chunk = torch.stack(
            [dataset[start_frame + step]["action"] for step in range(runner.chunk_size)])
        errors.append((predicted_chunk - recorded_chunk).abs().mean(dim=0))
        print(f"frame {start_frame:4d}  chunk {tuple(predicted_chunk.shape)}  latency {latencies[-1]:.2f} s")

    mean_error = torch.stack(errors).mean(dim=0)
    print(f"\ntask: {dataset[0]['task']}")
    if len(latencies) > 1:
        print(f"mean latency (first run excluded): {sum(latencies[1:]) / (len(latencies) - 1):.2f} s")
    print("mean absolute error per action:",
          ", ".join(f"{name} {value:.3f}" for name, value in zip(ACTION_NAMES, mean_error)))


if __name__ == "__main__":
    main()
