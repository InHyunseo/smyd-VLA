#include "vint_model.hpp"

#include <cstring>
#include <stdexcept>

VintModel::VintModel(const std::string & model_path, int inference_threads)
: environment_(ORT_LOGGING_LEVEL_WARNING, "vint"),
  session_(nullptr),
  memory_info_(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU))
{
  options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  options_.SetIntraOpNumThreads(inference_threads);
  session_ = Ort::Session(environment_, model_path.c_str(), options_);
  if (session_.GetInputCount() != 2 || session_.GetOutputCount() != 2) {
    throw std::runtime_error(model_path + " must have two inputs and two outputs");
  }

  Ort::AllocatorWithDefaultOptions allocator;
  for (size_t index = 0; index < session_.GetInputCount(); ++index) {
    input_names_.push_back(session_.GetInputNameAllocated(index, allocator).get());
  }
  for (size_t index = 0; index < session_.GetOutputCount(); ++index) {
    output_names_.push_back(session_.GetOutputNameAllocated(index, allocator).get());
  }
  for (const std::string & name : input_names_) {input_name_pointers_.push_back(name.c_str());}
  for (const std::string & name : output_names_) {output_name_pointers_.push_back(name.c_str());}

  observation_shape_ = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  const std::vector<int64_t> goal_shape =
    session_.GetInputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape();
  const std::vector<int64_t> waypoint_shape =
    session_.GetOutputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape();
  if (observation_shape_.size() != 4 || goal_shape.size() != 4 ||
    waypoint_shape.size() != 3 || observation_shape_[0] <= 0 ||
    observation_shape_[1] <= 0 || observation_shape_[2] <= 0 ||
    observation_shape_[3] != 3 || goal_shape[0] != -1 ||
    goal_shape[1] != observation_shape_[1] ||
    goal_shape[2] != observation_shape_[2] || goal_shape[3] != 3 ||
    waypoint_shape[1] <= 0 || waypoint_shape[2] != 4)
  {
    throw std::runtime_error(model_path + " has unexpected input or output shapes");
  }
  waypoint_count_ = static_cast<int>(waypoint_shape[1]);
  observation_buffer_.resize(
    static_cast<size_t>(context_length()) * image_height() * image_width() * 3);
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
  if (static_cast<int>(context_images.size()) != context_length() || goal_images.empty()) {
    throw std::runtime_error("wrong number of context or goal images");
  }
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
    Ort::RunOptions{nullptr}, input_name_pointers_.data(), inputs.data(), inputs.size(),
    output_name_pointers_.data(), output_name_pointers_.size());

  const size_t goal_count = goal_images.size();
  const float * distances = outputs[0].GetTensorData<float>();
  const float * waypoints = outputs[1].GetTensorData<float>();
  VintPrediction prediction;
  prediction.temporal_distances.assign(distances, distances + goal_count);
  prediction.waypoints.assign(
    waypoints, waypoints + goal_count * waypoint_count_ * 4);
  return prediction;
}
