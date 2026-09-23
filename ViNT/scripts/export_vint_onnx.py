#!/usr/bin/env python3
"""export_vint_onnx
ViNT 체크포인트를 전처리까지 포함한 ONNX 모델로 내보내고, PyTorch 출력과 같은지 확인한다.
C++ 노드가 픽셀 계산을 하지 않도록 resize와 정규화를 graph 안에 넣는다.

입력: cache/visualnav-transformer (공식 저장소), cache/model_weights/vint.pth
출력: model/vint.onnx, model/export_report.txt
"""

import argparse
import hashlib
import sys
from pathlib import Path

import numpy as np
import onnxruntime
import torch
import yaml
from PIL import Image

PROJECT_DIRECTORY = Path(__file__).resolve().parents[1]
REPOSITORY_DIRECTORY = PROJECT_DIRECTORY / "cache" / "visualnav-transformer"
CHECKPOINT_PATH = PROJECT_DIRECTORY / "cache" / "model_weights" / "vint.pth"
MODEL_DIRECTORY = PROJECT_DIRECTORY / "model"

IMAGE_MEAN = [0.485, 0.456, 0.406]
IMAGE_STANDARD_DEVIATION = [0.229, 0.224, 0.225]
OPSET_VERSION = 18
TOLERANCE = 1e-3


def load_vint(config):
    """공식 저장소의 ViNT 클래스를 만들고 체크포인트 가중치를 올린다."""
    sys.path.insert(0, str(REPOSITORY_DIRECTORY / "train"))
    from vint_train.models.vint.vint import ViNT

    model = ViNT(
        context_size=config["context_size"],
        len_traj_pred=config["len_traj_pred"],
        learn_angle=config["learn_angle"],
        obs_encoder=config["obs_encoder"],
        obs_encoding_size=config["obs_encoding_size"],
        late_fusion=config["late_fusion"],
        mha_num_attention_heads=config["mha_num_attention_heads"],
        mha_num_attention_layers=config["mha_num_attention_layers"],
        mha_ff_dim_factor=config["mha_ff_dim_factor"],
    )
    checkpoint = torch.load(CHECKPOINT_PATH, map_location="cpu", weights_only=False)
    loaded = checkpoint["model"]
    state_dict = loaded.module.state_dict() if hasattr(loaded, "module") else loaded.state_dict()
    model.load_state_dict(state_dict, strict=False)
    model.obs_encoder.set_swish(memory_efficient=False)   # 커스텀 autograd 함수는 export되지 않는다
    model.goal_encoder.set_swish(memory_efficient=False)
    return model.eval()


class ViNTWithPreprocessing(torch.nn.Module):
    """uint8 카메라 영상을 받아 ViNT 전처리를 graph 안에서 하고 시간 거리와 waypoint를 낸다."""

    def __init__(self, vint, image_width, image_height):
        """모델과 신경망 입력 크기를 기억하고 정규화 상수를 버퍼로 등록한다."""
        super().__init__()
        self.vint = vint
        self.network_size = (image_height, image_width)
        self.register_buffer("image_mean", torch.tensor(IMAGE_MEAN).reshape(1, 3, 1, 1))
        self.register_buffer("image_std", torch.tensor(IMAGE_STANDARD_DEVIATION).reshape(1, 3, 1, 1))

    def to_network_input(self, images):
        """uint8 [N, H, W, 3] 영상을 정규화된 float [N, 3, 64, 85]로 바꾼다."""
        values = images.permute(0, 3, 1, 2).float() / 255.0
        values = torch.nn.functional.interpolate(
            values, size=self.network_size, mode="bicubic", align_corners=False, antialias=True)
        return (values - self.image_mean) / self.image_std

    def forward(self, observation_images, goal_images):
        """관측 영상 6장과 목표 영상 여러 장으로 목표별 시간 거리와 waypoint를 예측한다."""
        observation = self.to_network_input(observation_images)
        goal = self.to_network_input(goal_images)
        observation = observation.reshape(1, -1, *self.network_size).expand(goal.shape[0], -1, -1, -1)
        return self.vint(observation, goal)


def make_images(count, height, width, seed):
    """검증에 쓸 uint8 영상 [count, H, W, 3]을 같은 값으로 다시 만들 수 있게 생성한다."""
    generator = torch.Generator().manual_seed(seed)
    return torch.randint(0, 256, (count, height, width, 3), dtype=torch.uint8, generator=generator)


def pil_pipeline_outputs(model, observation_images, goal_images, image_size):
    """상류 transform_images(PIL resize)를 그대로 거친 출력. 그래프 내장 전처리와 비교용이다."""
    def to_tensor(images):
        stacked = []
        for image in images.numpy():
            resized = np.asarray(Image.fromarray(image).resize(image_size), dtype=np.float32) / 255.0
            stacked.append(torch.from_numpy(resized).permute(2, 0, 1))
        values = torch.stack(stacked)
        mean = torch.tensor(IMAGE_MEAN).reshape(1, 3, 1, 1)
        standard_deviation = torch.tensor(IMAGE_STANDARD_DEVIATION).reshape(1, 3, 1, 1)
        return (values - mean) / standard_deviation

    observation = to_tensor(observation_images).reshape(1, -1, image_size[1], image_size[0])
    goal = to_tensor(goal_images)
    with torch.no_grad():
        return model(observation.expand(goal.shape[0], -1, -1, -1), goal)


def main():
    """체크포인트를 ONNX로 내보내고 PyTorch와 출력이 같은지 확인한 뒤 보고서를 쓴다."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-width", type=int, default=160)
    parser.add_argument("--camera-height", type=int, default=120)
    arguments = parser.parse_args()

    config = yaml.safe_load((REPOSITORY_DIRECTORY / "train" / "config" / "vint.yaml").read_text())
    image_size = tuple(config["image_size"])          # (width, height)
    context_length = config["context_size"] + 1
    model = ViNTWithPreprocessing(load_vint(config), *image_size).eval()

    observation = make_images(context_length, arguments.camera_height, arguments.camera_width, seed=0)
    goal = make_images(3, arguments.camera_height, arguments.camera_width, seed=1)
    MODEL_DIRECTORY.mkdir(parents=True, exist_ok=True)
    model_path = MODEL_DIRECTORY / "vint.onnx"
    torch.onnx.export(
        model, (observation, goal), str(model_path), dynamo=True, opset_version=OPSET_VERSION,
        input_names=["observation_images", "goal_images"],
        output_names=["temporal_distance", "waypoints"],
        dynamic_shapes={"observation_images": None,
                        "goal_images": {0: torch.export.Dim("goal_count", min=1, max=64)}})

    session = onnxruntime.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    input_names = [model_input.name for model_input in session.get_inputs()]
    report = [f"checkpoint: {CHECKPOINT_PATH.name} {CHECKPOINT_PATH.stat().st_size} bytes",
              f"sha256: {hashlib.sha256(CHECKPOINT_PATH.read_bytes()).hexdigest()}",
              f"camera: {arguments.camera_height}x{arguments.camera_width}, network: {image_size[1]}x{image_size[0]}",
              f"context_length: {context_length}, waypoints: {config['len_traj_pred']}",
              f"opset: {OPSET_VERSION}, tolerance: {TOLERANCE}"]

    for goal_count in (1, 5, 10):
        goals = make_images(goal_count, arguments.camera_height, arguments.camera_width, seed=goal_count)
        with torch.no_grad():
            torch_distance, torch_waypoints = model(observation, goals)
        onnx_distance, onnx_waypoints = session.run(
            None, dict(zip(input_names, [observation.numpy(), goals.numpy()])))
        distance_difference = np.abs(torch_distance.numpy() - onnx_distance).max()
        waypoint_difference = np.abs(torch_waypoints.numpy() - onnx_waypoints).max()
        report.append(f"goal_count {goal_count}: distance {distance_difference:.3e}, "
                      f"waypoint {waypoint_difference:.3e}")
        if max(distance_difference, waypoint_difference) > TOLERANCE:
            model_path.unlink()
            raise SystemExit(f"ONNX output mismatch (goal_count {goal_count}: distance "
                             f"{distance_difference:.3e}, waypoint {waypoint_difference:.3e})")

    with torch.no_grad():
        graph_distance, graph_waypoints = model(observation, goal)
    pil_distance, pil_waypoints = pil_pipeline_outputs(model.vint, observation, goal, image_size)
    report.append(f"PIL 전처리와의 차이 (참고): distance "
                  f"{np.abs(graph_distance.numpy() - pil_distance.numpy()).max():.3e}, "
                  f"waypoint {np.abs(graph_waypoints.numpy() - pil_waypoints.numpy()).max():.3e}")

    (MODEL_DIRECTORY / "export_report.txt").write_text("\n".join(report) + "\n")
    print("\n".join(report))
    print(f"saved {model_path}")


if __name__ == "__main__":
    main()
