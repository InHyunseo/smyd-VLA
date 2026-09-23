#!/usr/bin/env python3
"""overlay_closed_loop
closed-loop 평가의 raw 영상 위에 추론 정보를 겹친 영상을 따로 만든다(raw 영상은 그대로 둔다).
예측 궤적은 action의 위치 변화량(정규화 [-1, 1])에 한 스텝 이동 거리를 곱해 누적한 근사다.

입력: 실행 폴더 (evaluate_closed_loop.py 결과: videos/*/eval_episode_K.mp4, episode_K.npz)
출력: videos/*/eval_episode_K_overlay.mp4 (지나온 손끝 궤적, 예측한 앞으로 50스텝 궤적, 현재 손끝, 그리퍼 명령, 범례)
"""

import argparse
from pathlib import Path

import av
import cv2
import numpy as np

POSITION_STEP_METERS = 0.0125  # 위치 action 1.0에 대해 손끝이 한 스텝에 움직이는 거리 [m]
SCALE = 3  # 256 px 영상을 키워서 그린다.
EXECUTED_COLOR, PREDICTED_COLOR, END_EFFECTOR_COLOR = (255, 255, 255), (0, 200, 255), (255, 60, 60)  # RGB


def to_pixels(points, world_to_pixel, size):
    """월드 좌표 [N, 3]을 키운 영상의 픽셀 (x, y) [N, 2]로 바꾼다. 영상 기준으로 좌우를 되돌린다."""
    projected = np.c_[points, np.ones(len(points))] @ world_to_pixel.T
    column, row = projected[:, 0] / projected[:, 2], projected[:, 1] / projected[:, 2]
    return (np.stack([size - 1 - column, row], axis=1) * SCALE).astype(np.int32)


def draw_text(image, text, origin, scale=0.6):
    """반투명 검은 상자 위에 흰 글자를 그린다."""
    (width, height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = origin
    box = image[y - height - 4: y + baseline + 2, x - 4: x + width + 4]
    box[:] = (box * 0.4).astype(image.dtype)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)


def draw_frame(frame, step, record):
    """한 프레임에 궤적, 예측, 그리퍼 명령, 범례, 과제 문장을 그린다."""
    size = frame.shape[0]
    image = cv2.resize(frame, (size * SCALE, size * SCALE), interpolation=cv2.INTER_LINEAR)
    positions, chunks, world_to_pixel = (record["end_effector_positions"], record["predicted_chunks"],
                                         record["world_to_pixel"])
    cv2.polylines(image, [to_pixels(positions[: step + 1], world_to_pixel, size)], False, EXECUTED_COLOR, 2)
    if step < len(chunks):
        chunk = chunks[step]
        future = positions[step] + np.cumsum(chunk[:, :3] * POSITION_STEP_METERS, axis=0)
        predicted = to_pixels(np.vstack([positions[step], future]), world_to_pixel, size)
        cv2.polylines(image, [predicted], False, PREDICTED_COLOR, 2)
        for point in predicted[1::10]:
            cv2.circle(image, tuple(point), 3, PREDICTED_COLOR, -1)
        draw_text(image, f"Gripper: {'close' if chunk[0, 6] > 0 else 'open'}", (10, 50))
    current = to_pixels(positions[step: step + 1], world_to_pixel, size)[0]
    cv2.circle(image, tuple(current), 6, END_EFFECTOR_COLOR, -1)

    draw_text(image, str(record["task_description"]), (10, 25))
    draw_text(image, f"Step {step}", (image.shape[1] - 110, 50))
    legend = [("Executed", EXECUTED_COLOR), ("Predicted (50 steps, approx.)", PREDICTED_COLOR),
              ("End effector", END_EFFECTOR_COLOR)]
    for index, (label, color) in enumerate(legend):
        y = image.shape[0] - 20 - 25 * (len(legend) - 1 - index)
        cv2.line(image, (10, y - 5), (40, y - 5), color, 3)
        draw_text(image, label, (50, y))
    return image


def overlay(video_path, record_path):
    """raw 영상 한 편에 오버레이를 그려 *_overlay.mp4로 저장한다."""
    record = dict(np.load(record_path))
    output_path = video_path.with_name(f"{video_path.stem}_overlay.mp4")
    with av.open(str(video_path)) as source, av.open(str(output_path), "w") as target:
        source_stream = source.streams.video[0]
        stream = target.add_stream("libx264", rate=source_stream.average_rate)
        stream.width, stream.height = source_stream.width * SCALE, source_stream.height * SCALE
        stream.pix_fmt = "yuv420p"
        stream.options = {"crf": "18"}  # 글자와 선이 번지지 않을 만큼 화질을 높인다.
        frames = min(source_stream.frames, len(record["end_effector_positions"]))
        for step, frame in zip(range(frames), source.decode(video=0)):
            image = draw_frame(frame.to_ndarray(format="rgb24"), step, record)
            target.mux(stream.encode(av.VideoFrame.from_ndarray(image, format="rgb24")))
        target.mux(stream.encode())
    print(f"saved {output_path}")


def main():
    """실행 폴더 안의 영상마다 같은 번호의 기록으로 오버레이 영상을 만든다."""
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory", type=Path)
    args = parser.parse_args()
    for video_path in sorted(args.run_directory.glob("videos/*/eval_episode_*.mp4")):
        if video_path.stem.endswith("_overlay"):
            continue
        record_path = args.run_directory / f"episode_{video_path.stem.split('_')[-1]}.npz"
        if not record_path.exists():
            print(f"skipped {video_path.name}: {record_path.name} 없음")
            continue
        overlay(video_path, record_path)


if __name__ == "__main__":
    main()
