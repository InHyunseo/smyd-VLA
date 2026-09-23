#!/usr/bin/python3 -s
"""실행 폴더의 bag에서 카메라 영상을 H.264 MP4로 만든다.

폐루프는 오른쪽에 패널을 붙인 video_overlay.mp4도 만든다. 패널의 좌표는 카메라 화면에
투영한 것이 아니라 로봇 기준(x 앞, y 왼쪽)이다.

입력: 폐루프는 <실행 폴더>/bag, 개루프는 source_bag.txt가 가리키는 bag
출력: <실행 폴더>/video.mp4, 폐루프는 video_overlay.mp4도
"""

import argparse
import csv
import subprocess
from pathlib import Path

import cv2
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from rosidl_runtime_py.utilities import get_message

IMAGE_TOPIC = "/camera/image_raw"
WAYPOINT_TOPIC = "/waypoint"
NODE_TOPIC = "/vint/closest_node"


def read_errors(run):
    """개루프 오차를 (시각, 오차) 목록으로 읽는다. 폐루프면 빈 목록이다."""
    path = run / "waypoint_errors.csv"
    if not path.exists():
        return []
    with path.open(newline="") as stream:
        return [(float(row["seconds"]), float(row["error_m"])) for row in csv.DictReader(stream)]


def start_encoder(output, width=640):
    """임시 파일로 인코딩하는 ffmpeg 프로세스를 띄운다."""
    temporary = output.with_name(f"{output.stem}.tmp.mp4")
    process = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
         "-video_size", f"{width}x480", "-framerate", "4", "-i", "pipe:0", "-an",
         "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(temporary)],
        stdin=subprocess.PIPE,
    )
    return output, temporary, process


def draw_closed_overlay(frame, elapsed, waypoint, node):
    """카메라 프레임 오른쪽에 노드 번호와 경유점을 그린 패널을 붙인다."""
    image = cv2.copyMakeBorder(frame, 0, 0, 0, 320, cv2.BORDER_CONSTANT, value=(28, 32, 40))
    white, muted, cyan = (245, 245, 245), (168, 174, 182), (235, 190, 52)
    cv2.putText(image, "ViNT", (664, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.85, white, 2, cv2.LINE_AA)
    cv2.putText(image, f"CLOSED LOOP  |  {elapsed:.1f} s", (664, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.46, muted, 1, cv2.LINE_AA)
    cv2.putText(image, f"Topomap node  {node if node is not None else '--'}", (664, 108),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, white, 1, cv2.LINE_AA)
    cv2.line(image, (664, 128), (936, 128), (65, 70, 78), 1)
    cv2.putText(image, "Predicted waypoint", (664, 160),
                cv2.FONT_HERSHEY_SIMPLEX, 0.57, white, 1, cv2.LINE_AA)
    if waypoint is None:
        cv2.putText(image, "Waiting for inference", (664, 184),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, muted, 1, cv2.LINE_AA)
    else:
        cv2.putText(image, f"x {waypoint[0]:+.2f} m   y {waypoint[1]:+.2f} m", (664, 184),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, muted, 1, cv2.LINE_AA)
    origin = (800, 310)
    for radius in (40, 80):
        cv2.circle(image, origin, radius, (58, 63, 72), 1, cv2.LINE_AA)
    cv2.line(image, (800, 205), (800, 410), (77, 82, 90), 1)
    cv2.line(image, (695, 310), (905, 310), (77, 82, 90), 1)
    cv2.putText(image, "+x", (806, 218), cv2.FONT_HERSHEY_SIMPLEX, 0.4, muted, 1, cv2.LINE_AA)
    cv2.putText(image, "+y", (700, 303), cv2.FONT_HERSHEY_SIMPLEX, 0.4, muted, 1, cv2.LINE_AA)
    cv2.circle(image, origin, 5, white, -1, cv2.LINE_AA)
    if waypoint is not None:
        x, y = waypoint
        target = (int(origin[0] - y * 400), int(origin[1] - x * 400))
        cv2.arrowedLine(image, origin, target, cyan, 2, cv2.LINE_AA, tipLength=0.15)
        cv2.circle(image, target, 5, cyan, -1, cv2.LINE_AA)
    cv2.circle(image, (677, 448), 5, white, -1, cv2.LINE_AA)
    cv2.putText(image, "Robot", (691, 453), cv2.FONT_HERSHEY_SIMPLEX, 0.46, white, 1, cv2.LINE_AA)
    cv2.line(image, (782, 448), (802, 448), cyan, 2, cv2.LINE_AA)
    cv2.putText(image, "ViNT waypoint", (811, 453),
                cv2.FONT_HERSHEY_SIMPLEX, 0.46, white, 1, cv2.LINE_AA)
    return image


def main():
    """bag을 한 번 훑으면서 영상을 인코딩한다."""
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    source = run / "source_bag.txt"
    open_loop = source.exists()
    bag = Path(source.read_text().strip()) if open_loop else run / "bag"
    if not (bag / "metadata.yaml").is_file():
        raise RuntimeError(f"ROS bag not found: {bag}")

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag), storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
    reader.set_filter(StorageFilter(
        topics=[IMAGE_TOPIC] if open_loop else [IMAGE_TOPIC, WAYPOINT_TOPIC, NODE_TOPIC]))
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if IMAGE_TOPIC not in topic_types:
        raise RuntimeError(f"{IMAGE_TOPIC} is missing from {bag}")
    image_type = get_message(topic_types[IMAGE_TOPIC])
    encoders = [start_encoder(run / "video.mp4")]
    if not open_loop:
        encoders.append(start_encoder(run / "video_overlay.mp4", width=960))
    bridge = CvBridge()
    errors = read_errors(run) if open_loop else []
    error_index = 0
    first_stamp = None
    frames = 0
    latest_waypoint = None
    latest_node = None
    try:
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic == WAYPOINT_TOPIC:
                message = deserialize_message(data, get_message(topic_types[topic]))
                latest_waypoint = (
                    message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                    message.pose.position.x, message.pose.position.y)
                continue
            if topic == NODE_TOPIC:
                latest_node = deserialize_message(data, get_message(topic_types[topic])).data
                continue
            if topic != IMAGE_TOPIC:
                continue
            message = deserialize_message(data, image_type)
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if first_stamp is None:
                first_stamp = stamp
            frame = cv2.resize(bridge.imgmsg_to_cv2(message, desired_encoding="bgr8"), (640, 480))
            if open_loop:
                cv2.rectangle(frame, (0, 0), (640, 58), (20, 24, 32), -1)
                cv2.putText(frame, f"ViNT  OPEN LOOP  {stamp - first_stamp:.1f}s", (16, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                while error_index + 1 < len(errors) and errors[error_index + 1][0] <= stamp:
                    error_index += 1
                if errors and errors[error_index][0] <= stamp:
                    cv2.putText(frame, f"Waypoint error: {errors[error_index][1]:.2f} m", (16, 48),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (210, 225, 255), 1, cv2.LINE_AA)
            else:
                prediction = None
                if latest_waypoint is not None and 0 <= stamp - latest_waypoint[0] <= 2.0:
                    prediction = latest_waypoint[1:]
                overlay = draw_closed_overlay(frame.copy(), stamp - first_stamp, prediction, latest_node)
                encoders[1][2].stdin.write(overlay.tobytes())
            encoders[0][2].stdin.write(frame.tobytes())
            frames += 1
        if frames == 0:
            raise RuntimeError(f"no camera frames in {bag}")
        for _, _, encoder in encoders:
            encoder.stdin.close()
        for _, _, encoder in encoders:
            if encoder.wait() != 0:
                raise RuntimeError("ffmpeg failed to encode video")
        for output, temporary, _ in encoders:
            temporary.replace(output)
    finally:
        for _, temporary, encoder in encoders:
            if encoder.poll() is None:
                encoder.kill()
                encoder.wait()
            temporary.unlink(missing_ok=True)
    print(f"{run}: {frames} frames, {len(encoders)} video(s)")


if __name__ == "__main__":
    main()
