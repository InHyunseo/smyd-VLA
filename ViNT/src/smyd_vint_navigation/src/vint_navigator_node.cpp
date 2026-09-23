// vint_navigator_node
// 카메라 영상을 모아 ViNT에 넣고, topomap 후보 노드 중 시간 거리가 가장 짧은 곳을 현재 위치로 삼아
// 그 노드로 가는 경유점을 발행한다. 노드 선택 규칙은 상류 deployment/src/navigate.py와 같다.
//
// 입력: model_path, topomap_directory와 navigate.yaml 파라미터, camera/image_raw (Image)
// 출력: waypoint (PoseStamped, base_link 기준), topoplan/reached_goal (Bool),
//       vint/closest_node (Int32), vint/inference_seconds (Float32)

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include "vint_model.hpp"

namespace fs = std::filesystem;

// topomap 폴더의 0000.png부터를 번호 순서대로 RGB로 읽는다.
std::vector<cv::Mat> load_topomap(const fs::path & directory, int width, int height)
{
  if (!fs::is_directory(directory)) {
    throw std::runtime_error("topomap directory not found: " + directory.string());
  }
  std::vector<std::pair<int, fs::path>> files;
  for (const auto & entry : fs::directory_iterator(directory)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".png") {continue;}
    const std::string stem = entry.path().stem().string();
    if (stem.empty() || !std::all_of(stem.begin(), stem.end(),
      [](unsigned char character) {return std::isdigit(character) != 0;}))
    {
      throw std::runtime_error("non-numeric topomap image: " + entry.path().string());
    }
    files.emplace_back(std::stoi(stem), entry.path());
  }
  std::sort(files.begin(), files.end());
  if (files.empty()) {throw std::runtime_error("topomap has no PNG images");}

  std::vector<cv::Mat> images;
  images.reserve(files.size());
  for (size_t index = 0; index < files.size(); ++index) {
    if (files[index].first != static_cast<int>(index)) {
      throw std::runtime_error("topomap PNG indices must start at 0 without gaps");
    }
    cv::Mat image = cv::imread(files[index].second.string(), cv::IMREAD_COLOR);
    if (image.empty() || image.cols != width || image.rows != height) {
      throw std::runtime_error(
        "topomap image size differs from ONNX input: " + files[index].second.string());
    }
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    images.push_back(std::move(image));
  }
  return images;
}

// topomap을 따라가며 경유점을 발행하는 노드.
class VintNavigatorNode : public rclcpp::Node
{
public:
  // 모델과 topomap을 읽고 topic을 연결한다.
  VintNavigatorNode()
  : Node("vint_navigator_node")
  {
    const std::string model_path = declare_parameter<std::string>("model_path");
    const std::string topomap_directory = declare_parameter<std::string>("topomap_directory");
    const int inference_threads = declare_parameter<int>("inference_threads");
    const double inference_rate = declare_parameter<double>("inference_rate_hz");
    radius_ = declare_parameter<int>("radius");
    close_threshold_ = declare_parameter<double>("close_threshold");
    waypoint_index_ = declare_parameter<int>("waypoint_index");
    image_timeout_seconds_ = declare_parameter<double>("image_timeout_seconds");
    max_v_ = declare_parameter<double>("max_v");
    model_frame_rate_ = declare_parameter<double>("model_frame_rate");
    if (inference_threads < 0 || inference_rate <= 0 || radius_ < 0 ||
      image_timeout_seconds_ <= 0 || max_v_ <= 0 || model_frame_rate_ <= 0)
    {
      throw std::runtime_error("invalid navigator parameters");
    }
    model_ = std::make_unique<VintModel>(model_path, inference_threads);
    if (waypoint_index_ < 0 || waypoint_index_ >= model_->waypoint_count()) {
      throw std::runtime_error("waypoint_index is outside the model trajectory");
    }
    topomap_ = load_topomap(topomap_directory, model_->image_width(), model_->image_height());
    goal_node_ = static_cast<int>(topomap_.size()) - 1;

    waypoint_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("waypoint", 10);
    reached_publisher_ = create_publisher<std_msgs::msg::Bool>("topoplan/reached_goal", 10);
    closest_publisher_ = create_publisher<std_msgs::msg::Int32>("vint/closest_node", 10);
    inference_publisher_ = create_publisher<std_msgs::msg::Float32>("vint/inference_seconds", 10);
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {on_image(message);});
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / inference_rate));
    timer_ = create_wall_timer(period, [this]() {infer();});
    RCLCPP_INFO(get_logger(), "loaded %zu topomap nodes; ONNX input %dx%d, context %d",
      topomap_.size(), model_->image_width(), model_->image_height(), model_->context_length());
  }

private:
  // 카메라 프레임을 관측 큐에 넣는다. 큐 간격이 곧 모델이 보는 시간 간격이다.
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    const double stamp = rclcpp::Time(message->header.stamp).seconds();
    if (stamp < last_image_seconds_) {context_.clear();}          // bag을 다시 재생한 경우
    else if (std::isfinite(last_image_seconds_)) {
      const double interval = stamp - last_image_seconds_;
      if (std::abs(interval * model_frame_rate_ - 1.0) > 0.25) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
          "camera interval %.3fs does not match model_frame_rate %.1fHz", interval,
          model_frame_rate_);
      }
    }
    try {
      cv::Mat image = cv_bridge::toCvCopy(message, "rgb8")->image;
      if (image.cols != model_->image_width() || image.rows != model_->image_height()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
          "camera image size %dx%d differs from ONNX input %dx%d", image.cols, image.rows,
          model_->image_width(), model_->image_height());
        return;
      }
      context_.push_back(std::move(image));
      if (context_.size() > static_cast<size_t>(model_->context_length())) {context_.pop_front();}
      last_image_seconds_ = stamp;
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "camera conversion failed: %s", error.what());
    }
  }

  // 예측 경유점 하나를 base_link 기준 자세로 바꾼다. 위치는 MAX_V / RATE로 미터가 된다.
  geometry_msgs::msg::PoseStamped waypoint_pose(
    const VintPrediction & prediction, size_t goal_offset, const rclcpp::Time & stamp) const
  {
    const size_t offset = (goal_offset * model_->waypoint_count() + waypoint_index_) * 4;
    const double yaw =
      std::atan2(prediction.waypoints[offset + 3], prediction.waypoints[offset + 2]);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = prediction.waypoints[offset] * max_v_ / model_frame_rate_;
    pose.pose.position.y = prediction.waypoints[offset + 1] * max_v_ / model_frame_rate_;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    return pose;
  }

  // 추론 주기마다 도착 여부를 알리고, 관측이 준비된 동안에는 한 스텝씩 진행한다.
  void infer()
  {
    const bool ready = !reached_goal_ &&
      context_.size() == static_cast<size_t>(model_->context_length()) &&
      now().seconds() - last_image_seconds_ <= image_timeout_seconds_;
    if (ready) {step();}
    std_msgs::msg::Bool reached;
    reached.data = reached_goal_;
    reached_publisher_->publish(reached);
  }

  // 후보 노드를 한 번에 추론해 현재 노드를 갱신하고 경유점을 발행한다.
  void step()
  {
    const int start = std::max(0, closest_node_ - radius_);
    const int end = std::min(goal_node_, closest_node_ + radius_ + 1);
    const std::vector<cv::Mat> goals(topomap_.begin() + start, topomap_.begin() + end + 1);
    const std::vector<cv::Mat> observations(context_.begin(), context_.end());
    const auto beginning = std::chrono::steady_clock::now();
    VintPrediction prediction;
    try {
      prediction = model_->predict(observations, goals);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "ViNT inference failed: %s", error.what());
      return;
    }
    std_msgs::msg::Float32 duration;
    duration.data = std::chrono::duration<float>(std::chrono::steady_clock::now() - beginning).count();
    inference_publisher_->publish(duration);

    if (!std::all_of(prediction.temporal_distances.begin(), prediction.temporal_distances.end(),
      [](float value) {return std::isfinite(value);}))
    {
      RCLCPP_ERROR(get_logger(), "ViNT returned a non-finite temporal distance");
      return;
    }
    const size_t closest_offset = static_cast<size_t>(std::distance(
      prediction.temporal_distances.begin(),
      std::min_element(
        prediction.temporal_distances.begin(), prediction.temporal_distances.end())));
    const bool advance = prediction.temporal_distances[closest_offset] <= close_threshold_;
    const size_t subgoal_offset = std::min(closest_offset + advance, goals.size() - 1);
    closest_node_ = advance ?
      std::min(goal_node_, start + static_cast<int>(closest_offset) + 1) :
      start + static_cast<int>(closest_offset);
    std_msgs::msg::Int32 closest;
    closest.data = closest_node_;
    closest_publisher_->publish(closest);

    reached_goal_ = closest_node_ == goal_node_;
    if (!reached_goal_) {
      waypoint_publisher_->publish(waypoint_pose(prediction, subgoal_offset, now()));
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "node=%d/%d distance=%.2f inference=%.3fs", closest_node_, goal_node_,
      prediction.temporal_distances[closest_offset], duration.data);
  }

  std::unique_ptr<VintModel> model_;
  std::vector<cv::Mat> topomap_;
  std::deque<cv::Mat> context_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr closest_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr inference_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double last_image_seconds_ = -std::numeric_limits<double>::infinity();
  double image_timeout_seconds_;
  double close_threshold_;
  double max_v_;
  double model_frame_rate_;
  int radius_;
  int waypoint_index_;
  int goal_node_;
  int closest_node_ = 0;
  bool reached_goal_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VintNavigatorNode>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "vint_navigator_node: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
