"""smolvla_policy
smolvla_node와 evaluate_open_loop가 함께 쓰는 SmolVLA 로딩·추론 모듈.

입력: Hugging Face 모델 id, LeRobot 형식 관측 dict
      (이미지 CHW float [0, 1] 2장, 상태 8차원(손끝 위치·자세·그리퍼), task 문장)
출력: action chunk 텐서 [chunk_size, 7] (손끝 위치·자세 변화량과 그리퍼 명령)
"""

import torch
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy


class SmolVLAPolicyRunner:
    """모델과 전처리·후처리를 한 번 불러 두고 chunk 단위로 추론한다."""

    def __init__(self, model_id):
        """체크포인트에 들어 있는 전처리·후처리(정규화 통계 포함)를 그대로 쓴다."""
        self.policy = SmolVLAPolicy.from_pretrained(model_id).to("cpu").eval()
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            self.policy.config, pretrained_path=model_id,
            preprocessor_overrides={"device_processor": {"device": "cpu"}})
        self.chunk_size = self.policy.config.chunk_size

    @torch.no_grad()
    def predict_chunk(self, observation):
        """관측 하나로 action chunk [chunk_size, 7]을 예측한다."""
        return self.postprocessor(self.policy.predict_action_chunk(self.preprocessor(observation)))[0]
