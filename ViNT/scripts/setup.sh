#!/usr/bin/env bash
# setup.sh
# 가상환경을 만들고, ViNT 공식 저장소와 체크포인트를 cache/에 받은 뒤, 모델을 ONNX로 내보낸다.
# ROS 의존성(rosdep)과 빌드(colcon)는 MANUAL.md의 명령을 따로 실행한다.
#
# 입력: 없음
# 출력: .venv/, cache/ (공식 저장소와 vint.pth), model/ (vint.onnx, export_report.txt)

set -euo pipefail
cd "$(dirname "$0")/.."

CHECKPOINT_URL="https://drive.google.com/uc?id=1ckrceGb5m_uUtq3pD8KHwnqtJgPl6kF5"
CHECKPOINT_SHA256="155fd72de2e98ae0e2fef9404072e1aefa79dae5f7f2411d4bcf7e384b83aa1f"

python3 -m venv .venv
touch .venv/COLCON_IGNORE          # colcon이 .venv 안을 빌드하지 않도록
source .venv/bin/activate
pip install --quiet --upgrade pip
pip install --quiet --requirement requirements.txt

# 모델 정의만 필요하므로 공식 저장소는 clone만 하고 설치하지 않는다.
if [ ! -d cache/visualnav-transformer ]; then
  git clone --depth 1 https://github.com/robodhruv/visualnav-transformer cache/visualnav-transformer
fi
mkdir -p cache && touch cache/COLCON_IGNORE          # colcon이 내려받은 저장소를 빌드하지 않도록

# 체크포인트 다운로드에 실패하면 브라우저로 받아 같은 경로에 두고 다시 실행한다.
if [ ! -f cache/model_weights/vint.pth ] || \
   ! printf '%s  %s\n' "$CHECKPOINT_SHA256" cache/model_weights/vint.pth | sha256sum --check --status; then
  mkdir -p cache/model_weights
  gdown "$CHECKPOINT_URL" -O cache/model_weights/vint.pth.part
  printf '%s  %s\n' "$CHECKPOINT_SHA256" cache/model_weights/vint.pth.part | sha256sum --check
  mv cache/model_weights/vint.pth.part cache/model_weights/vint.pth
fi

python scripts/export_vint_onnx.py

echo "done. 다음: source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -y && colcon build --symlink-install"
