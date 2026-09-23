#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class WaypointFollowerNode : public rclcpp::Node
{
public:
  WaypointFollowerNode()
  : Node("waypoint_follower_node")
  {
    max_v_ = declare_parameter<double>("max_v", 0.2);
    max_w_ = declare_parameter<double>("max_w", 0.4);
    model_frame_rate_ = declare_parameter<double>("model_frame_rate", 4.0);
    timeout_seconds_ = declare_parameter<double>("waypoint_timeout_seconds", 2.0);
    const double control_rate = declare_parameter<double>("control_rate_hz", 10.0);
    if (max_v_ <= 0 || max_w_ <= 0 || model_frame_rate_ <= 0 ||
      timeout_seconds_ <= 0 || control_rate <= 0)
    {
      throw std::runtime_error("invalid follower parameters");
    }
    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    waypoint_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "waypoint", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr waypoint) {
        if (waypoint->header.frame_id != "base_link") {
          RCLCPP_ERROR(get_logger(), "waypoint must be in base_link, got %s",
            waypoint->header.frame_id.c_str());
          return;
        }
        waypoint_ = *waypoint;
        last_waypoint_time_ = std::chrono::steady_clock::now();
        have_waypoint_ = true;
      });
    goal_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "topoplan/reached_goal", 10,
      [this](const std_msgs::msg::Bool::SharedPtr goal) {
        if (goal->data) {reached_goal_ = true;}
      });
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate));
    timer_ = create_wall_timer(period, [this]() {publish_command();});
  }

  void stop() {command_publisher_->publish(geometry_msgs::msg::Twist());}

private:
  void publish_command()
  {
    geometry_msgs::msg::Twist command;
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_waypoint_time_).count();
    if (reached_goal_ || !have_waypoint_ || age > timeout_seconds_) {
      command_publisher_->publish(command);
      return;
    }

    const double dx = waypoint_.pose.position.x;
    const double dy = waypoint_.pose.position.y;
    constexpr double epsilon = 1e-8;
    double linear = 0.0;
    double angular = 0.0;
    if (std::abs(dx) < epsilon && std::abs(dy) < epsilon) {
      const auto & orientation = waypoint_.pose.orientation;
      const double yaw = std::atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
      angular = yaw * model_frame_rate_;
    } else if (std::abs(dx) < epsilon) {
      angular = std::copysign(M_PI / 2.0, dy) * model_frame_rate_;
    } else {
      linear = dx * model_frame_rate_;
      angular = std::atan(dy / dx) * model_frame_rate_;
    }
    command.linear.x = std::clamp(linear, 0.0, max_v_);
    command.angular.z = std::clamp(angular, -max_w_, max_w_);
    command_publisher_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped waypoint_;
  std::chrono::steady_clock::time_point last_waypoint_time_;
  double max_v_ = 0.2;
  double max_w_ = 0.4;
  double model_frame_rate_ = 4.0;
  double timeout_seconds_ = 2.0;
  bool have_waypoint_ = false;
  bool reached_goal_ = false;
};

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int)
{
  stop_requested = 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  try {
    auto node = std::make_shared<WaypointFollowerNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (!stop_requested && rclcpp::ok()) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    node->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } catch (const std::exception & error) {
    std::fprintf(stderr, "waypoint_follower_node: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
