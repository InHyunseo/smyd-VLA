#!/usr/bin/python3 -s
"""Save one comparison figure from an open- or closed-loop ViNT run."""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def values(data, key):
    return [float(row[key]) for row in data]


def plot_open_loop(run):
    """Figure 1. ViNT의 0.75초 뒤 경유점과 기록된 이동 사이의 2D 위치 오차.

    CSV의 각 행은 비교 가능한 예측 하나이며, 시간 0은 첫 예측 시각이다.
    같은 시연을 topomap에도 사용하므로 독립 테스트 결과가 아니다.
    """
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
    """Figure 1. 기록 경로와 폐루프 주행 궤적, 그리고 목표까지 남은 거리.

    왼쪽 검은 점선은 topomap 기록 시의 odom 경로, 파란 선은 ViNT가 주행한 odom 경로다.
    별은 기록 경로의 마지막 위치다. 오른쪽은 첫 odom 이후의 목표 거리이며 빨간 점선은
    위치 기준 성공 반경 0.5 m다. 이 그림만으로 영상 기반 노드 인식의 정확도는 판단하지 않는다.
    """
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
    y_values = values(recorded, "y") + values(trajectory, "y")
    axes[0].set_ylim(min(y_values) - 0.08, max(y_values) + 0.08)
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
