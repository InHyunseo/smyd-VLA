#!/usr/bin/env python3
"""plot_open_loop
기록한 bag에서 녹화된 action과 예측 chunk를 읽어 그림 한 장을 저장하고 성분별 오차를 출력한다.

입력: 실행 폴더 (open_loop.launch.py record:=true로 만든 results/open_loop/<시각>_episode_NNNNNN, 안에 bag/)
출력: 같은 실행 폴더의 figure1.png
"""

import argparse
from pathlib import Path

import matplotlib
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from rosidl_runtime_py.utilities import get_message

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ACTION_NAMES = ["x", "y", "z", "roll", "pitch", "yaw", "gripper"]
MOTION_NAMES = ACTION_NAMES[:-1]  # 손끝 위치·자세 (그리퍼 제외)
GROUND_TRUTH_TOPIC = "/ground_truth/action"
CHUNK_TOPIC = "/prediction/chunk"


def read_bag(bag):
    """두 topic의 (메시지 시각 [s], 메시지) 목록을 읽는다. 시각은 보낸 쪽이 찍은 sim time이다."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag), storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
    topics = [GROUND_TRUTH_TOPIC, CHUNK_TOPIC]
    reader.set_filter(StorageFilter(topics=topics))
    types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()
             if topic.name in topics}
    messages = {topic: [] for topic in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        message = deserialize_message(data, types[topic])
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        messages[topic].append((stamp, message))
    return messages


def ground_truth_series(messages, start):
    """녹화된 action을 시각 [s]과 값 [N, 7] 배열로 바꾼다. 같은 시각이면 마지막 값만 둔다."""
    times = np.array([stamp for stamp, _ in messages])
    values = np.array([message.values for _, message in messages])
    _, reversed_index = np.unique(times[::-1], return_index=True)
    keep = len(times) - 1 - reversed_index
    keep = keep[times[keep] >= start]
    return times[keep] - start, values[keep]


def chunk_series(messages, start):
    """예측 chunk를 이어 붙여 시각 [s]과 값 [N, 7] 배열로 바꾼다(스텝마다 한 점)."""
    times, values = [], []
    for stamp, message in messages:
        chunk = np.array(message.actions).reshape(-1, message.action_dimension)
        times.extend(stamp - start + step * message.seconds_per_step for step in range(len(chunk)))
        values.extend(chunk)
    return np.array(times), np.array(values)


def summarize_predictions(prediction, time_range):
    """같은 시각을 덮는 chunk 예측들의 중앙값과 사분위 범위를 구한다."""
    times, values = prediction
    in_range = (times >= time_range[0]) & (times <= time_range[1])
    times = np.round(times[in_range], decimals=6)  # 부동소수점 오차가 있는 같은 스텝을 묶는다.
    values = values[in_range]
    order = np.argsort(times)
    unique_times, starts = np.unique(times[order], return_index=True)
    groups = np.split(values[order], starts[1:])
    quartiles = np.stack([np.percentile(group, [25, 50, 75], axis=0) for group in groups])
    return unique_times, quartiles[:, 1], quartiles[:, 0], quartiles[:, 2]


def horizon_errors(ground_truth, messages, start):
    """chunk마다 스텝별 |예측 - 녹화| 를 구해 chunk 전체 평균을 낸다. 결과: [스텝, 성분]."""
    errors = []
    for stamp, message in messages:
        chunk = np.array(message.actions).reshape(-1, message.action_dimension)
        targets = stamp - start + np.arange(len(chunk)) * message.seconds_per_step
        if targets[-1] > ground_truth[0][-1]:
            continue  # 에피소드 끝이나 기록 끝에서 잘린 chunk
        recorded = np.stack([np.interp(targets, ground_truth[0], values) for values in ground_truth[1].T], axis=1)
        errors.append(np.abs(chunk - recorded))
    if not errors:
        return np.zeros((0, 0)), 0
    return np.mean(errors, axis=0), len(errors)


def plot_actions(ground_truth, prediction, output):
    """Figure 1. action 성분별 시간 변화.

    LIBERO의 action 7개(손끝 위치 변화량 x·y·z, 자세 변화량 roll·pitch·yaw, 그리퍼 명령)를 sim time 축으로
    그린다. 검은 실선(Ground truth)은 데이터셋에 녹화된 시연자의 action이다. 파란 실선은 같은 시각을 덮는
    SmolVLA 예측 chunk들의 중앙값이고, 음영은 25--75 백분위 범위다. 값은 제어기 입력 단위로 [-1, 1]
    범위이며, 시간은 첫 예측의 기준 관측 시각을 0으로 한다.
    """
    summary = summarize_predictions(prediction, (ground_truth[0][0], ground_truth[0][-1]))
    figure, axes = plt.subplots(4, 2, figsize=(7.2, 7), sharex=True, constrained_layout=True)
    legend_handles = None
    for index, (axis, name) in enumerate(zip(axes.flat, ACTION_NAMES)):
        band = axis.fill_between(
            summary[0], summary[2][:, index], summary[3][:, index],
            color="tab:blue", alpha=0.18, linewidth=0, label="Prediction IQR")
        predicted, = axis.plot(
            summary[0], summary[1][:, index], color="tab:blue", linewidth=1.15,
            label="Prediction median")
        truth, = axis.plot(
            ground_truth[0], ground_truth[1][:, index], color="black", linewidth=1.15,
            label="Ground truth")
        if legend_handles is None:
            legend_handles = [truth, predicted, band]
        axis.axhline(0, color="0.86", linewidth=0.6, zorder=0)
        axis.set_title(name, loc="left", fontsize=9, fontweight="semibold", pad=3)
        axis.set_xlim(ground_truth[0][0], ground_truth[0][-1])
        axis.margins(x=0)
        axis.tick_params(labelsize=8, length=3)
        axis.spines[["top", "right"]].set_visible(False)

    axes.flat[-1].axis("off")
    axes.flat[-1].legend(
        legend_handles, ["Ground truth", "Prediction median", "Prediction IQR (25--75%)"],
        loc="center", frameon=False, fontsize=8)
    for axis in (axes[-1, 0], axes[-2, 1]):
        axis.tick_params(labelbottom=True)
    figure.supxlabel("Time [s]", fontsize=9)
    figure.supylabel("Action", fontsize=9)
    save(figure, output / "figure1")


def save(figure, path):
    """그림을 PNG로 저장한다."""
    figure.savefig(path.with_suffix(".png"), dpi=300)
    plt.close(figure)


def main():
    """bag을 읽어 그림 한 장을 저장하고 요약 수치를 출력한다."""
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory", type=Path)
    args = parser.parse_args()

    messages = read_bag(args.run_directory / "bag")
    chunks = messages[CHUNK_TOPIC]
    if not chunks or not messages[GROUND_TRUTH_TOPIC]:
        raise SystemExit(f"{CHUNK_TOPIC} 또는 {GROUND_TRUTH_TOPIC} 메시지가 bag에 없습니다")
    start = chunks[0][0]  # 첫 예측의 관측 시각. 그 전(모델 로딩 대기)은 버린다.
    ground_truth = ground_truth_series(messages[GROUND_TRUTH_TOPIC], start)
    prediction = chunk_series(chunks, start)
    errors, chunk_count = horizon_errors(ground_truth, chunks, start)
    if chunk_count == 0:
        raise SystemExit("녹화된 구간 안에서 끝까지 실행된 chunk가 없습니다")

    plot_actions(ground_truth, prediction, args.run_directory)
    print(f"chunks: {chunk_count} complete / {len(chunks)} total")
    motion = errors[:, :len(MOTION_NAMES)]
    print("mean absolute error per action:",
          ", ".join(f"{name} {value:.3f}" for name, value in zip(ACTION_NAMES, errors.mean(axis=0))))
    print(f"손끝 오차 평균: 스텝 1에서 {motion[0].mean():.3f}, 스텝 {len(errors)}에서 {motion[-1].mean():.3f}")
    print(f"saved {args.run_directory.resolve()}")


if __name__ == "__main__":
    main()
