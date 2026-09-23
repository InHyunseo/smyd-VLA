#!/usr/bin/env python3
"""episode_replay_node
LIBERO 데이터셋 에피소드를 로봇과 카메라처럼 chunk 단위로 재생하고, 시뮬레이션 시계(/clock)를 낸다.
한 프레임에서 시계를 멈추고 관측을 1초마다 다시 보내다가, 예측 chunk가 오면 녹화된 다음 N 프레임을 재생한다.
그래서 use_sim_time 노드(RViz2 등)는 추론 동안 함께 멈추고, 시간축에는 재생한 구간만 남는다.
loop가 false면 에피소드를 한 번 재생하고 노드를 끝낸다(launch 전체가 함께 종료된다).

입력: dataset, episode 또는 task (에피소드 고르기), steps_per_inference (chunk에서 재생할 스텝 수),
      camera_keys (데이터셋마다 다른 카메라 항목 이름), frames_per_second, loop 파라미터,
      prediction/chunk (ActionChunk)
출력: /clock, observation/image, observation/image2 (Image, rgb8), observation/state (StateVector, 손끝 위치 3 + axis-angle 3 + 그리퍼 2),
      instruction (String, latched), ground_truth/action (StateVector, 매 프레임),
      ground_truth/image, ground_truth/image2 (Image, 매 프레임), ground_truth/path (Marker, 지나온 손끝 궤적),
      TF world → end_effector
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from lerobot.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Image
from smyd_vla_msgs.msg import ActionChunk, StateVector
from std_msgs.msg import String
from visualization_msgs.msg import Marker

CAMERA_TOPICS = ["image", "image2"]  # 모델이 쓰는 카메라 두 대 (정면, 손목)
START_SECONDS = 1.0  # 시뮬레이션 시계 시작 시각 [s]


class EpisodeReplayNode(Node):
    """녹화된 에피소드를 재생하고 예측 chunk와 박자를 맞춘다."""

    def __init__(self):
        """파라미터로 에피소드를 고르고 topic을 연결한 뒤 첫 chunk를 시작한다."""
        super().__init__("episode_replay_node")
        dataset_id = self.declare_parameter("dataset", "HuggingFaceVLA/libero").value
        episode = self.declare_parameter("episode", 0).value
        task = self.declare_parameter("task", "").value
        self.loop = self.declare_parameter("loop", True).value
        self.frames_per_second = self.declare_parameter("frames_per_second", 10.0).value
        self.steps_per_inference = self.declare_parameter("steps_per_inference", 1).value
        camera_keys = self.declare_parameter(
            "camera_keys", ["observation.images.image", "observation.images.image2"]).value
        if len(camera_keys) != len(CAMERA_TOPICS):
            raise ValueError(f"camera_keys must have {len(CAMERA_TOPICS)} entries, got {len(camera_keys)}")
        self.cameras = dict(zip(CAMERA_TOPICS, camera_keys))

        if task:
            episode = self.find_episode(dataset_id, task)
        # 0번부터 범위로 불러온 뒤 해당 에피소드의 프레임만 메모리에 올린다.
        dataset = LeRobotDataset(dataset_id, episodes=list(range(episode + 1)))
        indices = [index for index, value in enumerate(dataset.hf_dataset["episode_index"])
                   if int(value) == episode]
        if not indices:
            raise ValueError(f"episode {episode} not available in {dataset_id}")
        self.frames = [self.to_frame(dataset[index]) for index in indices]
        self.task = dataset[indices[0]]["task"]
        self.get_logger().info(f'episode {episode}, {len(self.frames)} frames, task: "{self.task}"')

        self.clock_publisher = self.create_publisher(Clock, "/clock", 10)
        self.instruction_publisher = self.create_publisher(
            String, "instruction", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.image_publishers = {name: self.create_publisher(Image, f"observation/{name}", 1)
                                 for name in self.cameras}
        self.ground_truth_image_publishers = {name: self.create_publisher(Image, f"ground_truth/{name}", 1)
                                              for name in self.cameras}
        self.state_publisher = self.create_publisher(StateVector, "observation/state", 1)
        self.action_publisher = self.create_publisher(StateVector, "ground_truth/action", 10)
        self.path_publisher = self.create_publisher(Marker, "ground_truth/path", 1)
        self.transform_broadcaster = TransformBroadcaster(self)
        self.create_subscription(ActionChunk, "prediction/chunk", self.on_chunk, 10)
        self.create_timer(1.0 / self.frames_per_second, self.on_timer)

        self.simulation_time = START_SECONDS
        self.current_frame, self.frames_left_to_play, self.ticks_since_observation = 0, 0, 0
        self.path = []  # 지나온 손끝 위치
        self.instruction_publisher.publish(String(data=self.task))
        self.publish_ground_truth()
        self.start_chunk()

    def to_frame(self, item):
        """데이터셋 한 프레임을 카메라 이미지(rgb8 바이트), 상태, action으로 바꾼다."""
        images = {name: (item[key].permute(1, 2, 0).numpy() * 255).round().astype(np.uint8)
                  for name, key in self.cameras.items()}
        return {"images": images, "state": item["observation.state"].tolist(), "action": item["action"].tolist()}

    def find_episode(self, dataset_id, task):
        """과제 설명에 task가 들어가는 첫 에피소드 번호를 찾는다."""
        episodes = LeRobotDatasetMetadata(dataset_id).episodes
        for index, tasks in enumerate(episodes["tasks"]):
            if any(task.lower() in description.lower() for description in tasks):
                return index
        raise ValueError(f'no episode with task containing "{task}"')

    def stamp(self):
        """현재 시뮬레이션 시각."""
        return Time(seconds=self.simulation_time).to_msg()

    def to_image_message(self, image, camera, stamp):
        """rgb8 배열을 Image 메시지로 만든다."""
        message = Image(height=image.shape[0], width=image.shape[1], encoding="rgb8",
                        step=image.shape[1] * 3, data=image.tobytes())
        message.header.stamp, message.header.frame_id = stamp, camera
        return message

    def publish_observation(self):
        """모델 입력(카메라 2장 + 상태)을 관측 시각으로 보낸다."""
        frame = self.frames[self.current_frame]
        for name, image in frame["images"].items():
            self.image_publishers[name].publish(self.to_image_message(image, name, self.observation_stamp))
        state = StateVector(values=frame["state"])
        state.header.stamp = self.observation_stamp
        self.state_publisher.publish(state)

    def publish_ground_truth(self):
        """현재 프레임의 카메라 화면, 녹화된 action, 지나온 손끝 궤적을 보낸다."""
        frame = self.frames[self.current_frame]
        for name, image in frame["images"].items():
            self.ground_truth_image_publishers[name].publish(self.to_image_message(image, name, self.stamp()))
        action = StateVector(values=frame["action"])
        action.header.stamp = self.stamp()
        self.action_publisher.publish(action)

        position = frame["state"][:3]
        self.path.append(Point(x=position[0], y=position[1], z=position[2]))
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id = self.stamp(), "world"
        transform.child_frame_id = "end_effector"
        transform.transform.translation.x, transform.transform.translation.y = position[0], position[1]
        transform.transform.translation.z, transform.transform.rotation.w = position[2], 1.0
        self.transform_broadcaster.sendTransform(transform)
        if len(self.path) > 1:
            marker = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns="ground_truth", points=self.path)
            marker.header.frame_id, marker.header.stamp = "world", self.stamp()
            marker.scale.x, marker.color.a = 0.004, 1.0
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.8, 0.1
            self.path_publisher.publish(marker)

    def start_chunk(self):
        """현재 시각으로 이 프레임의 관측을 보내고 예측을 기다린다."""
        self.observation_stamp = self.stamp()
        self.ticks_since_observation = 0
        self.publish_observation()

    def on_chunk(self, message):
        """지금 관측에 대한 예측이면 steps_per_inference만큼 재생을 시작한다."""
        if self.frames_left_to_play == 0 and message.header.stamp == self.observation_stamp:
            steps = len(message.actions) // message.action_dimension
            self.frames_left_to_play = min(self.steps_per_inference, steps)

    def on_timer(self):
        """재생 중이면 시계와 프레임을 한 칸 넘기고, 대기 중이면 1초마다 관측을 다시 보낸다."""
        playing = self.frames_left_to_play > 0
        if playing:
            self.simulation_time += 1.0 / self.frames_per_second
            self.frames_left_to_play -= 1
            self.current_frame += 1
            if self.current_frame >= len(self.frames):
                if not self.loop:
                    self.get_logger().info("episode finished")
                    raise SystemExit
                self.get_logger().info("episode finished, restarting")
                self.current_frame, self.frames_left_to_play, self.path = 0, 0, []
        else:
            self.ticks_since_observation += 1
            if self.ticks_since_observation >= self.frames_per_second:
                self.ticks_since_observation = 0
                self.publish_observation()
        self.clock_publisher.publish(Clock(clock=self.stamp()))
        if playing:
            self.publish_ground_truth()
        if playing and self.frames_left_to_play == 0:
            self.start_chunk()


def main():
    """Ctrl+C나 에피소드 종료에 traceback 없이 끝낸다."""
    rclpy.init()
    try:
        rclpy.spin(EpisodeReplayNode())
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    except Exception as error:
        rclpy.logging.get_logger("episode_replay_node").fatal(str(error))
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
