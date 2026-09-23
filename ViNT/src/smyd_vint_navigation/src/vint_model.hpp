// vint_model
// ONNX으로 내보낸 ViNT를 한 번 열어두고, 관측 영상과 목표 영상들을 넣어 추론한다.
// 영상 크기 조정과 정규화는 ONNX 그래프 안에 있으므로 여기서는 픽셀을 복사만 한다.
//
// 입력: model/vint.onnx 경로, 추론 스레드 수, RGB 8비트 영상
// 출력: 목표별 시간 거리와 경유점

#ifndef VINT_MODEL_HPP_
#define VINT_MODEL_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

// 목표 영상 하나당 시간 거리 1개와 경유점 waypoint_count()개(dx, dy, cos, sin).
struct VintPrediction
{
  std::vector<float> temporal_distances;
  std::vector<float> waypoints;
};

// ViNT ONNX 세션.
class VintModel
{
public:
  // 모델을 열고 입출력 shape이 기대와 맞는지 확인한다.
  VintModel(const std::string & model_path, int inference_threads);

  // 관측 영상 context_length()장과 목표 영상들로 추론한다.
  VintPrediction predict(
    const std::vector<cv::Mat> & context_images, const std::vector<cv::Mat> & goal_images);

  int image_width() const {return static_cast<int>(observation_shape_[2]);}
  int image_height() const {return static_cast<int>(observation_shape_[1]);}
  int context_length() const {return static_cast<int>(observation_shape_[0]);}
  int waypoint_count() const {return waypoint_count_;}

private:
  // 영상들을 buffer에 이어 붙인다.
  void copy_images(const std::vector<cv::Mat> & images, std::vector<uint8_t> & buffer) const;

  Ort::Env environment_;
  Ort::SessionOptions options_;
  Ort::Session session_;
  Ort::MemoryInfo memory_info_;
  std::vector<int64_t> observation_shape_;
  std::vector<uint8_t> observation_buffer_;
  std::vector<uint8_t> goal_buffer_;
  int waypoint_count_;
};

#endif  // VINT_MODEL_HPP_
