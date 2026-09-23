#include "vint_model.hpp"

#include <cstring>
#include <iterator>
#include <stdexcept>

namespace
{
// scripts/export_vint_onnx.py가 고정한 이름.
constexpr const char * INPUT_NAMES[] = {"observation_images", "goal_images"};
constexpr const char * OUTPUT_NAMES[] = {"temporal_distance", "waypoints"};
}  // namespace

VintModel::VintModel(const std::string & model_path, int inference_threads)
: environment_(ORT_LOGGING_LEVEL_WARNING, "vint"),
  session_(nullptr),
  memory_info_(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU))
{
  options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  options_.SetIntraOpNumThreads(inference_threads);
  session_ = Ort::Session(environment_, model_path.c_str(), options_);

  observation_shape_ = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  const std::vector<int64_t> goal_shape =
    session_.GetInputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape();
  const std::vector<int64_t> waypoint_shape =
    session_.GetOutputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape();
  if (observation_shape_.size() != 4 || waypoint_shape.size() != 3 ||
    waypoint_shape[2] != 4 ||
    goal_shape != std::vector<int64_t>{
      -1, observation_shape_[1], observation_shape_[2], 3})
  {
    throw std::runtime_error(model_path + " has unexpected input or output shapes");
  }
  waypoint_count_ = static_cast<int>(waypoint_shape[1]);
}

void VintModel::copy_images(
  const std::vector<cv::Mat> & images, std::vector<uint8_t> & buffer) const
{
  const size_t image_bytes = static_cast<size_t>(image_height()) * image_width() * 3;
  buffer.resize(images.size() * image_bytes);
  for (size_t index = 0; index < images.size(); ++index) {
    const cv::Mat & image = images[index];
    if (image.type() != CV_8UC3 || image.cols != image_width() || image.rows != image_height() ||
      !image.isContinuous())
    {
      throw std::runtime_error("image must be a continuous CV_8UC3 of the model input size");
    }
    std::memcpy(buffer.data() + index * image_bytes, image.data, image_bytes);
  }
}

VintPrediction VintModel::predict(
  const std::vector<cv::Mat> & context_images, const std::vector<cv::Mat> & goal_images)
{
  copy_images(context_images, observation_buffer_);
  copy_images(goal_images, goal_buffer_);
  const std::vector<int64_t> goal_shape{
    static_cast<int64_t>(goal_images.size()), image_height(), image_width(), 3};

  std::vector<Ort::Value> inputs;
  inputs.push_back(
    Ort::Value::CreateTensor<uint8_t>(
      memory_info_, observation_buffer_.data(), observation_buffer_.size(),
      observation_shape_.data(), observation_shape_.size()));
  inputs.push_back(
    Ort::Value::CreateTensor<uint8_t>(
      memory_info_, goal_buffer_.data(), goal_buffer_.size(), goal_shape.data(), goal_shape.size()));

  const std::vector<Ort::Value> outputs = session_.Run(
    Ort::RunOptions{nullptr}, INPUT_NAMES, inputs.data(), inputs.size(),
    OUTPUT_NAMES, std::size(OUTPUT_NAMES));

  const size_t goal_count = goal_images.size();
  const float * distances = outputs[0].GetTensorData<float>();
  const float * waypoints = outputs[1].GetTensorData<float>();
  VintPrediction prediction;
  prediction.temporal_distances.assign(distances, distances + goal_count);
  prediction.waypoints.assign(waypoints, waypoints + goal_count * waypoint_count_ * 4);
  return prediction;
}
