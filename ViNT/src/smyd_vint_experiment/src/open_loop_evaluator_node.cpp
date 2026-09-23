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

struct PendingWaypoint
{
  double stamp;
  double x;
  double y;
  double yaw;
  double predicted_x;
  double predicted_y;
};

class OpenLoopEvaluatorNode : public rclcpp::Node
{
public:
  OpenLoopEvaluatorNode()
  : Node("open_loop_evaluator_node")
  {
    const fs::path output_directory = declare_parameter<std::string>("output_directory", "");
    if (output_directory.empty()) {throw std::runtime_error("output_directory is required");}
    fs::create_directories(output_directory);
    output_.open(output_directory / "waypoint_errors.csv");
    if (!output_) {throw std::runtime_error("cannot open waypoint_errors.csv");}
    output_ << "seconds,predicted_dx_m,predicted_dy_m,recorded_dx_m,recorded_dy_m,error_m\n";

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {on_odometry(message);});
    waypoint_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "waypoint", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {on_waypoint(message);});
  }

  void finish()
  {
    output_.close();
    RCLCPP_INFO(get_logger(), "compared %zu waypoints", samples_);
  }

private:
  // The configured waypoint index is 2 and ViNT predicts at 4 Hz.
  static constexpr double horizon_seconds_ = 0.75;

  void on_waypoint(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!have_odometry_) {return;}
    const double stamp = rclcpp::Time(message->header.stamp).seconds();
    if (stamp <= last_odometry_stamp_ - 0.2) {return;}
    pending_.push_back({stamp, last_x_, last_y_, last_yaw_,
      message->pose.position.x, message->pose.position.y});
  }

  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const double stamp = rclcpp::Time(message->header.stamp).seconds();
    const double x = message->pose.pose.position.x;
    const double y = message->pose.pose.position.y;
    const auto & q = message->pose.pose.orientation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    if (have_odometry_ && stamp < last_odometry_stamp_) {pending_.clear();}
    last_odometry_stamp_ = stamp;
    last_x_ = x;
    last_y_ = y;
    last_yaw_ = yaw;
    have_odometry_ = true;

    while (!pending_.empty() && stamp >= pending_.front().stamp + horizon_seconds_) {
      const auto prediction = pending_.front();
      pending_.pop_front();
      const double dx = x - prediction.x;
      const double dy = y - prediction.y;
      const double recorded_x = std::cos(prediction.yaw) * dx + std::sin(prediction.yaw) * dy;
      const double recorded_y = -std::sin(prediction.yaw) * dx + std::cos(prediction.yaw) * dy;
      const double error = std::hypot(
        prediction.predicted_x - recorded_x, prediction.predicted_y - recorded_y);
      output_ << prediction.stamp << "," << prediction.predicted_x << ","
              << prediction.predicted_y << "," << recorded_x << "," << recorded_y << ","
              << error << "\n";
      ++samples_;
    }
  }

  std::ofstream output_;
  std::deque<PendingWaypoint> pending_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_subscription_;
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
