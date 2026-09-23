// open_loop_evaluator_node
// 기록된 주행을 재생하는 동안 ViNT가 낸 경유점을, 그 뒤 실제로 기록된 odom 이동과 비교한다.
// 로봇은 움직이지 않으므로 예측이 주행에 영향을 주지 않는다.
//
// 입력: output_directory와 navigate.yaml 파라미터, waypoint (PoseStamped), odom (Odometry)
// 출력: output_directory/waypoint_errors.csv

#include <cmath>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fs = std::filesystem;

// 아직 비교 시점이 되지 않은 예측 하나와 그때의 로봇 자세.
struct PendingWaypoint
{
  double stamp;
  double x;
  double y;
  double yaw;
  double predicted_x;
  double predicted_y;
};

// 쿼터니언에서 yaw만 꺼낸다.
double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// 경유점 예측과 이후 이동을 비교해 CSV로 남기는 노드.
class OpenLoopEvaluatorNode : public rclcpp::Node
{
public:
  // 비교 지평을 파라미터에서 구하고 CSV를 연다.
  OpenLoopEvaluatorNode()
  : Node("open_loop_evaluator_node")
  {
    const fs::path output_directory = declare_parameter<std::string>("output_directory");
    horizon_seconds_ = (declare_parameter<int>("waypoint_index") + 1) /
      declare_parameter<double>("model_frame_rate");
    fs::create_directories(output_directory);
    output_.open(output_directory / "waypoint_errors.csv");
    if (!output_) {throw std::runtime_error("cannot open waypoint_errors.csv");}
    output_ << "seconds,predicted_dx_m,predicted_dy_m,recorded_dx_m,recorded_dy_m,error_m\n";

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {on_odometry(message);});
    waypoint_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "waypoint", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        if (!have_odometry_) {return;}
        pending_.push_back({rclcpp::Time(message->header.stamp).seconds(), last_x_, last_y_,
          last_yaw_, message->pose.position.x, message->pose.position.y});
      });
  }

  // 비교한 예측 수를 남긴다.
  void finish()
  {
    output_.close();
    RCLCPP_INFO(get_logger(), "compared %zu waypoints", samples_);
  }

private:
  // 현재 자세를 갱신하고, 지평이 지난 예측을 실제 이동과 비교해 한 줄씩 적는다.
  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const double stamp = rclcpp::Time(message->header.stamp).seconds();
    const double x = message->pose.pose.position.x;
    const double y = message->pose.pose.position.y;
    if (have_odometry_ && stamp < last_odometry_stamp_) {pending_.clear();}
    last_odometry_stamp_ = stamp;
    last_x_ = x;
    last_y_ = y;
    last_yaw_ = yaw_of(message->pose.pose.orientation);
    have_odometry_ = true;

    while (!pending_.empty() && stamp >= pending_.front().stamp + horizon_seconds_) {
      const PendingWaypoint prediction = pending_.front();
      pending_.pop_front();
      const double dx = x - prediction.x;
      const double dy = y - prediction.y;
      const double recorded_x = std::cos(prediction.yaw) * dx + std::sin(prediction.yaw) * dy;
      const double recorded_y = -std::sin(prediction.yaw) * dx + std::cos(prediction.yaw) * dy;
      output_ << prediction.stamp << "," << prediction.predicted_x << ","
              << prediction.predicted_y << "," << recorded_x << "," << recorded_y << ","
              << std::hypot(prediction.predicted_x - recorded_x, prediction.predicted_y - recorded_y)
              << "\n";
      output_.flush();
      ++samples_;
    }
  }

  std::ofstream output_;
  std::deque<PendingWaypoint> pending_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_subscription_;
  double horizon_seconds_;
  double last_odometry_stamp_ = 0.0;
  double last_x_ = 0.0;
  double last_y_ = 0.0;
  double last_yaw_ = 0.0;
  size_t samples_ = 0;
  bool have_odometry_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<OpenLoopEvaluatorNode>();
    rclcpp::spin(node);
    node->finish();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "open_loop_evaluator_node: %s\n", error.what());
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return 0;
}
