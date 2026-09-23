#!/usr/bin/python3 -s
"""실행 폴더 하나에서 figure1.png를 만든다.

입력: 개루프는 waypoint_errors.csv, 폐루프는 trajectory.csv와 source_topomap.txt
출력: <실행 폴더>/figure1.png
"""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def rows(path):
    """CSV를 dict 목록으로 읽는다."""
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def values(data, key):
    """열 하나를 float 목록으로 꺼낸다."""
    return [float(row[key]) for row in data]


def plot_open_loop(run):
    """예측 경유점과 실제 이동 사이의 거리 오차를 재생 시간축에 그린다."""
    data = rows(run / "waypoint_errors.csv")
    if not data:
        raise RuntimeError("waypoint_errors.csv has no comparable predictions")
    time = values(data, "seconds")
    time = [value - time[0] for value in time]
    figure, axis = plt.subplots(figsize=(7, 3.5), constrained_layout=True)
    axis.plot(time, values(data, "error_m"), color="#2563a6", linewidth=1.7)
    axis.set(xlabel="Replay time [s]", ylabel="Waypoint error [m]", title="Open-loop waypoint error")
    axis.set_xlim(time[0], time[-1])
    axis.set_ylim(bottom=0)
    axis.grid(alpha=0.18)
    axis.spines[["top", "right"]].set_visible(False)
    return figure


def plot_closed_loop(run):
    """왼쪽에 기록 경로와 주행 궤적을, 오른쪽에 목표까지 거리와 성공 반경을 그린다."""
    trajectory = rows(run / "trajectory.csv")
    if not trajectory:
        raise RuntimeError("trajectory.csv has no odometry")
    topomap = Path((run / "source_topomap.txt").read_text().strip())
    recorded = rows(topomap / "poses.csv")
    figure, axes = plt.subplots(1, 2, figsize=(9, 4), constrained_layout=True)
    axes[0].plot(values(recorded, "x"), values(recorded, "y"), "k--", label="Recorded route")
    axes[0].plot(values(trajectory, "x"), values(trajectory, "y"), color="#2563a6", label="ViNT drive")
    axes[0].scatter([float(recorded[-1]["x"])], [float(recorded[-1]["y"])],
                    color="#c24132", marker="*", s=100, label="Goal")
    axes[0].set(xlabel="x [m]", ylabel="y [m]", title="Route")
    axes[0].set_aspect("equal")
    axes[0].legend(frameon=False, fontsize=8)
    axes[1].plot(values(trajectory, "seconds"), values(trajectory, "distance_to_goal_m"),
                 color="#2563a6")
    axes[1].axhline(0.5, color="#c24132", linestyle="--", linewidth=1, label="Success radius")
    axes[1].set(xlabel="Time [s]", ylabel="Distance [m]", title="Distance to goal")
    axes[1].legend(frameon=False, fontsize=8)
    for axis in axes:
        axis.grid(alpha=0.2)
    figure.suptitle("Closed-loop navigation")
    return figure


def main():
    """실행 폴더 종류를 보고 맞는 그림을 저장한다."""
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    figure = plot_open_loop(run) if (run / "waypoint_errors.csv").exists() else plot_closed_loop(run)
    output = run / "figure1.png"
    figure.savefig(output, dpi=180)
    plt.close(figure)
    print(output)


if __name__ == "__main__":
    main()
