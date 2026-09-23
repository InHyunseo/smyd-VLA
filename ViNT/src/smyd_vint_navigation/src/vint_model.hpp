#ifndef VINT_MODEL_HPP_
#define VINT_MODEL_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

struct VintPrediction
{
  std::vector<float> temporal_distances;
  std::vector<float> waypoints;
};

class VintModel
{
public:
  VintModel(const std::string & model_path, int inference_threads);

  VintPrediction predict(
    const std::vector<cv::Mat> & context_images, const std::vector<cv::Mat> & goal_images);

  int image_width() const {return static_cast<int>(observation_shape_[2]);}
  int image_height() const {return static_cast<int>(observation_shape_[1]);}
  int context_length() const {return static_cast<int>(observation_shape_[0]);}
  int waypoint_count() const {return waypoint_count_;}
private:
  void copy_images(const std::vector<cv::Mat> & images, std::vector<uint8_t> & buffer) const;

  Ort::Env environment_;
  Ort::SessionOptions options_;
  Ort::Session session_;
  Ort::MemoryInfo memory_info_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<const char *> input_name_pointers_;
  std::vector<const char *> output_name_pointers_;
  std::vector<int64_t> observation_shape_;
  std::vector<uint8_t> observation_buffer_;
  std::vector<uint8_t> goal_buffer_;
  int waypoint_count_;
};

#endif  // VINT_MODEL_HPP_
