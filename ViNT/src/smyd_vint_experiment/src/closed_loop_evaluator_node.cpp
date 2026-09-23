#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

namespace fs = std::filesystem;

struct Route
{
  double length = 0.0;
  double goal_x = 0.0;
  double goal_y = 0.0;
};

Route read_route(const fs::path & csv)
{
  std::ifstream input(csv);
  if (!input) {throw std::runtime_error("cannot read " + csv.string());}
  std::string line;
  std::getline(input, line);
  Route route;
  int count = 0;
  while (std::getline(input, line)) {
    if (line.empty()) {continue;}
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    int index;
    double x, y, yaw, seconds;
    if (!(row >> index >> x >> y >> yaw >> seconds) || index != count ||
      !std::isfinite(x) || !std::isfinite(y))
    {
      throw std::runtime_error("invalid poses.csv row: " + line);
    }
    if (count > 0) {route.length += std::hypot(x - route.goal_x, y - route.goal_y);}
    route.goal_x = x;
    route.goal_y = y;
    ++count;
  }
  if (count == 0) {throw std::runtime_error("poses.csv has no poses");}
  return route;
}

class ClosedLoopEvaluatorNode : public rclcpp::Node
{
public:
  ClosedLoopEvaluatorNode()
  : Node("closed_loop_evaluator_node")
  {
    const fs::path topomap = declare_parameter<std::string>("topomap_directory", "");
    output_directory_ = declare_parameter<std::string>("output_directory", "");
    if (topomap.empty() || output_directory_.empty()) {
      throw std::runtime_error("topomap_directory and output_directory are required");
    }
    route_ = read_route(topomap / "poses.csv");
    fs::create_directories(output_directory_);
    trajectory_file_.open(output_directory_ / "trajectory.csv");
    if (!trajectory_file_) {throw std::runtime_error("cannot open trajectory.csv");}
    trajectory_file_ << "seconds,x,y,distance_to_goal_m\n";

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {on_odometry(message);});
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {on_scan(message);});
    goal_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "topoplan/reached_goal", 10,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        if (message->data && !model_claimed_goal_ && have_position_) {
          model_claimed_goal_ = true;
          model_claimed_goal_distance_m_ = distance_to_goal(last_x_, last_y_);
        }
      });
  }

  bool finished() const {return finished_;}

  void finish(const std::string & reason)
  {
    if (finished_) {return;}
    finished_ = true;
    trajectory_file_.flush();
    const fs::path temporary = output_directory_ / "result.yaml.tmp";
    std::ofstream output(temporary);
    if (!output) {throw std::runtime_error("cannot write " + temporary.string());}
    output << std::boolalpha << std::setprecision(6);
    output << "success: " << (reason == "success") << "\n";
    output << "termination_reason: " << reason << "\n";
    output << "elapsed_seconds: " << elapsed_seconds_ << "\n";
    output << "recorded_path_length_m: " << route_.length << "\n";
    output << "traveled_path_length_m: " << traveled_length_m_ << "\n";
    if (route_.length > 0) {
      output << "path_length_ratio: " << traveled_length_m_ / route_.length << "\n";
    } else {
      output << "path_length_ratio: null\n";
    }
    output << "final_distance_to_goal_m: " << final_distance_m_ << "\n";
    output << "collision_events: " << collision_events_ << "\n";
    output << "scan_samples: " << scan_samples_ << "\n";
    output << "model_claimed_goal: " << model_claimed_goal_ << "\n";
    if (model_claimed_goal_) {
      output << "model_claimed_goal_distance_m: " << model_claimed_goal_distance_m_ << "\n";
    } else {
      output << "model_claimed_goal_distance_m: null\n";
    }
    output.close();
    if (!output) {throw std::runtime_error("cannot finish " + temporary.string());}
    fs::rename(temporary, output_directory_ / "result.yaml");
    RCLCPP_INFO(get_logger(), "saved result: %s, distance %.2fm", reason.c_str(), final_distance_m_);
  }

private:
  static constexpr double success_radius_m_ = 0.5;
  static constexpr double time_limit_seconds_ = 300.0;
  static constexpr double collision_distance_m_ = 0.2;

  double distance_to_goal(double x, double y) const
  {
    return std::hypot(x - route_.goal_x, y - route_.goal_y);
  }

  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (finished_) {return;}
    const double stamp = rclcpp::Time(message->header.stamp).seconds();
    const double x = message->pose.pose.position.x;
    const double y = message->pose.pose.position.y;
    if (!have_position_) {
      start_seconds_ = stamp;
      have_position_ = true;
    } else {
      traveled_length_m_ += std::hypot(x - last_x_, y - last_y_);
    }
    last_x_ = x;
    last_y_ = y;
    elapsed_seconds_ = std::max(0.0, stamp - start_seconds_);
    final_distance_m_ = distance_to_goal(x, y);
    trajectory_file_ << elapsed_seconds_ << "," << x << "," << y << ","
                     << final_distance_m_ << "\n";
    if (final_distance_m_ <= success_radius_m_) {
      finish("success");
      rclcpp::shutdown();
    } else if (elapsed_seconds_ >= time_limit_seconds_) {
      finish("timeout");
      rclcpp::shutdown();
    }
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    ++scan_samples_;
    double nearest = std::numeric_limits<double>::infinity();
    for (float range : message->ranges) {
      if (std::isfinite(range) && range >= message->range_min && range <= message->range_max) {
        nearest = std::min(nearest, static_cast<double>(range));
      }
    }
    const bool near_obstacle = nearest < collision_distance_m_;
    if (near_obstacle && !near_obstacle_previous_) {++collision_events_;}
    near_obstacle_previous_ = near_obstacle;
  }

  Route route_;
  fs::path output_directory_;
  std::ofstream trajectory_file_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_subscription_;
  double start_seconds_ = 0.0;
  double last_x_ = 0.0;
  double last_y_ = 0.0;
  double elapsed_seconds_ = 0.0;
  double traveled_length_m_ = 0.0;
  double final_distance_m_ = std::numeric_limits<double>::infinity();
  double model_claimed_goal_distance_m_ = 0.0;
  int collision_events_ = 0;
  int scan_samples_ = 0;
  bool have_position_ = false;
  bool near_obstacle_previous_ = false;
  bool model_claimed_goal_ = false;
  bool finished_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ClosedLoopEvaluatorNode>();
    rclcpp::spin(node);
    if (!node->finished()) {node->finish("interrupted");}
  } catch (const std::exception & error) {
    std::fprintf(stderr, "closed_loop_evaluator_node: %s\n", error.what());
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return 0;
}
