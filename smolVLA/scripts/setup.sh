#!/usr/bin/env bash
# setup.sh
# 프로젝트 가상환경을 만들고 Python 의존성을 설치한다. 저장소 폴더에서 한 번만 실행하면 된다.
# ROS 의존성(rosdep)과 빌드(colcon)는 MANUAL.md의 명령을 따로 실행한다.
#
# 입력: 없음 (--model 을 주면 SmolVLA 체크포인트도 미리 받는다)
# 출력: .venv/ (가상환경), Hugging Face 캐시의 모델 파일

set -euo pipefail
cd "$(dirname "$0")/.."

python3 -m venv .venv
touch .venv/COLCON_IGNORE          # colcon이 .venv 안을 빌드하지 않도록
source .venv/bin/activate
pip install --quiet --upgrade pip

# egl_probe(LIBERO 의존성)는 venv 안의 cmake로는 빌드되지 않아, 시스템 cmake로 먼저 설치한다.
PATH=/usr/bin:$PATH pip install --quiet --no-build-isolation egl_probe hf-egl-probe
pip install --quiet --requirement requirements.txt

# LIBERO는 처음 import할 때 데이터 경로를 묻는다. 기본값으로 답해 설정 파일을 만들어 둔다.
echo N | python -c "import libero.libero" > /dev/null 2>&1 || true

if [[ "${1:-}" == "--model" ]]; then
  python - <<'PYTHON'
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
SmolVLAPolicy.from_pretrained("HuggingFaceVLA/smolvla_libero")
print("model downloaded")
PYTHON
fi

echo "done. 다음: source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -y && colcon build --symlink-install"
