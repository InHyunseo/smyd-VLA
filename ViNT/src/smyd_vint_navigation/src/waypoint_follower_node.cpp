// waypoint_follower_node
// ViNT가 낸 경유점을 상류 deployment/src/pd_controller.py와 같은 식으로 속도 명령으로 바꾼다.
// 경유점이 끊기거나 목표에 도착하면 정지 명령을 낸다.
//
// 입력: navigate.yaml 파라미터, waypoint (PoseStamped), topoplan/reached_goal (Bool)
// 출력: cmd_vel (Twist)

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

// 경유점을 cmd_vel로 바꾸는 노드.
class WaypointFollowerNode : public rclcpp::Node
{
public:
  // 파라미터를 읽고 제어 주기 타이머를 건다.
  WaypointFollowerNode()
  : Node("waypoint_follower_node")
  {
    max_v_ = declare_parameter<double>("max_v");
    max_w_ = declare_parameter<double>("max_w");
    model_frame_rate_ = declare_parameter<double>("model_frame_rate");
    timeout_seconds_ = declare_parameter<double>("waypoint_timeout_seconds");
    const double control_rate = declare_parameter<double>("control_rate_hz");
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

  // 정지 명령을 한 번 낸다.
  void stop() {command_publisher_->publish(geometry_msgs::msg::Twist());}

private:
  // 마지막 경유점으로 속도를 계산해 발행한다.
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
      angular = 2.0 * std::atan2(orientation.z, orientation.w) * model_frame_rate_;
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
  double max_v_;
  double max_w_;
  double model_frame_rate_;
  double timeout_seconds_;
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

// rclcpp의 신호 처리를 끄고 직접 받는다. 종료 뒤에도 정지 명령을 한 번 더 내야 하기 때문이다.
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
      executor.spin_once(std::chrono::milliseconds(10));
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
