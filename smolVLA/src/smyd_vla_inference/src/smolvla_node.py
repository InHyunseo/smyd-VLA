#!/usr/bin/env python3
"""smolvla_node
같은 stamp의 관측(카메라 2장 + 상태)이 모이면 SmolVLA로 action chunk를 예측해 보낸다.
추론은 별도 스레드에서 돌고, 추론 중에 들어온 관측은 건너뛴다.

입력: model_id, frames_per_second 파라미터, observation/image, observation/image2 (Image, rgb8),
      observation/state (StateVector), instruction (String, latched)
출력: prediction/chunk (ActionChunk, 관측 stamp를 그대로 씀),
      prediction/path (Marker, 예측대로 움직였을 때의 손끝 궤적 근사)
"""

import signal
import threading
import time

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from smyd_vla_msgs.msg import ActionChunk, StateVector
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from smolvla_policy import SmolVLAPolicyRunner

OBSERVATION_TOPICS = {"image": (Image, "observation/image"), "image2": (Image, "observation/image2"),
                      "state": (StateVector, "observation/state")}
POSITION_STEP_METERS = 0.0125  # 위치 action 1.0에 대해 손끝이 한 스텝에 움직이는 거리 [m]


class SmolVLANode(Node):
    """관측을 모아 추론 스레드에 넘기고 결과를 chunk와 궤적 마커로 보낸다."""

    def __init__(self):
        """파라미터를 읽고 모델을 불러온 뒤 topic을 연결한다."""
        super().__init__("smolvla_node")
        model_id = self.declare_parameter("model_id", "HuggingFaceVLA/smolvla_libero").value
        self.frames_per_second = self.declare_parameter("frames_per_second", 10.0).value

        self.get_logger().info(f"loading {model_id} ...")
        self.runner = SmolVLAPolicyRunner(model_id)
        self.get_logger().info("model ready")

        self.latest = dict.fromkeys(OBSERVATION_TOPICS)
        self.instruction = None
        self.last_used_stamp = None
        self.busy = False
        self.inference_thread = None
        self.chunk_publisher = self.create_publisher(ActionChunk, "prediction/chunk", 10)
        self.path_publisher = self.create_publisher(Marker, "prediction/path", 1)
        for key, (message_type, topic) in OBSERVATION_TOPICS.items():
            self.create_subscription(
                message_type, topic, lambda message, key=key: self.on_observation(key, message), 1)
        self.create_subscription(
            String, "instruction", self.on_instruction,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_instruction(self, message):
        """latched 명령을 저장한다."""
        self.instruction = message.data
        self.get_logger().info(f'instruction: "{self.instruction}"')

    def on_observation(self, key, message):
        """세 관측의 stamp가 같고 아직 안 쓴 것이며 추론 중이 아니면 추론 스레드를 시작한다."""
        self.latest[key] = message
        stamps = [value.header.stamp for value in self.latest.values() if value is not None]
        if (self.busy or self.instruction is None or len(stamps) < len(OBSERVATION_TOPICS)
                or stamps.count(stamps[0]) < len(stamps) or stamps[0] == self.last_used_stamp):
            return
        self.busy, self.last_used_stamp = True, stamps[0]
        self.inference_thread = threading.Thread(
            target=self.run_inference, args=(dict(self.latest), self.instruction))
        self.inference_thread.start()

    def run_inference(self, messages, instruction):
        """관측을 모델 입력으로 바꿔 추론하고, chunk와 예측 궤적을 보낸다."""
        try:
            state = np.array(messages["state"].values, dtype=np.float32)
            observation = {
                "observation.images.image": to_image_tensor(messages["image"]),
                "observation.images.image2": to_image_tensor(messages["image2"]),
                "observation.state": torch.from_numpy(state),
                "task": instruction,
            }
            start_time = time.perf_counter()
            chunk = self.runner.predict_chunk(observation).numpy()
            latency = time.perf_counter() - start_time
            if not rclpy.ok():
                return

            message = ActionChunk(seconds_per_step=1.0 / self.frames_per_second,
                                  action_dimension=chunk.shape[1], actions=chunk.flatten().tolist())
            message.header.stamp = messages["state"].header.stamp
            self.chunk_publisher.publish(message)
            self.publish_path(state[:3], chunk, messages["state"].header.stamp)
            self.get_logger().info(f"published chunk {chunk.shape}, latency {latency:.2f} s")
        except Exception as error:
            self.get_logger().error(f"inference failed: {error}")
            self.last_used_stamp = None
        finally:
            self.busy = False

    def publish_path(self, position, chunk, stamp):
        """예측한 위치 변화량을 누적해 손끝이 지나갈 경로를 마커로 그린다(근사)."""
        future = position + np.cumsum(chunk[:, :3] * POSITION_STEP_METERS, axis=0)
        marker = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns="prediction",
                        points=[Point(x=float(x), y=float(y), z=float(z)) for x, y, z in future])
        marker.header.frame_id, marker.header.stamp = "world", stamp
        marker.scale.x, marker.color.a, marker.color.r, marker.color.g, marker.color.b = 0.004, 1.0, 0.1, 0.7, 1.0
        self.path_publisher.publish(marker)


def to_image_tensor(message):
    """rgb8 Image 메시지를 CHW float [0, 1] 텐서로 바꾼다."""
    image = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3)
    return torch.from_numpy(image.copy()).permute(2, 0, 1).float() / 255.0


def main():
    """Ctrl+C나 launch 종료 신호를 받으면 진행 중인 추론을 마치고 traceback 없이 끝낸다."""
    rclpy.init()
    node = None
    try:
        node = SmolVLANode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None and node.inference_thread is not None:
            node.inference_thread.join()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
