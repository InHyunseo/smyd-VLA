// topomap_recorder_node
// 사람이 로봇을 몰고 다니는 동안 카메라 영상을 일정 간격으로 저장하고 그때의 odom 위치를 함께 적는다.
// 저장한 폴더가 ViNT의 topological map이 되고, 마지막 줄의 위치가 주행 평가의 목표가 된다.
//
// 입력: output_directory, seconds_per_node, meters_per_node 파라미터,
//       camera/image_raw (Image), odom (Odometry)
// 출력: output_directory/NNNN.png, output_directory/poses.csv

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 카메라 영상과 odom 위치를 topomap 폴더로 저장한다.
class TopomapRecorderNode : public rclcpp::Node
{
public:
  // 저장 폴더와 간격을 파라미터로 받고 topic을 연결한다.
  TopomapRecorderNode()
  : Node("topomap_recorder_node")
  {
    output_directory_ = declare_parameter<std::string>("output_directory", "topomap");
    seconds_per_node_ = declare_parameter<double>("seconds_per_node", 1.0);
    meters_per_node_ = declare_parameter<double>("meters_per_node", 0.05);
    if (seconds_per_node_ <= 0.0 || meters_per_node_ <= 0.0) {
      throw std::runtime_error("seconds_per_node and meters_per_node must be positive");
    }
    std::filesystem::create_directories(output_directory_);
    poses_file_.open(output_directory_ / "poses.csv");
    if (!poses_file_) {throw std::runtime_error("cannot open poses.csv");}
    poses_file_ << "index,x,y,yaw,seconds\n";

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {last_odometry_ = message;});
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {on_image(message);});
  }

private:
  // 시간 간격이 지났고 로봇이 그만큼 움직였으면 노드 하나를 저장한다.
  // 멈춰 있는 동안 같은 영상이 쌓이면 주행 때 노드 선택이 앞질러 나간다.
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    if (last_odometry_ == nullptr) {return;}
    const double seconds = rclcpp::Time(message->header.stamp).seconds();
    const auto & position = last_odometry_->pose.pose.position;
    if (saved_count_ > 0 &&
      (seconds - last_saved_seconds_ < seconds_per_node_ ||
      std::hypot(position.x - last_saved_x_, position.y - last_saved_y_) < meters_per_node_))
    {
      return;
    }
    char name[16];
    std::snprintf(name, sizeof(name), "%04d.png", saved_count_);
    if (!cv::imwrite(
        (output_directory_ / name).string(), cv_bridge::toCvShare(message, "bgr8")->image))
    {
      RCLCPP_ERROR(get_logger(), "cannot save %s", name);
      return;
    }

    poses_file_ << saved_count_ << "," << position.x << "," << position.y << ","
                << tf2::getYaw(last_odometry_->pose.pose.orientation) << "," << seconds << "\n";
    poses_file_.flush();          // Ctrl+C로 끝내도 기록이 남도록
    last_saved_seconds_ = seconds;
    last_saved_x_ = position.x;
    last_saved_y_ = position.y;
    saved_count_ += 1;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "topomap nodes: %d", saved_count_);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  nav_msgs::msg::Odometry::SharedPtr last_odometry_;
  std::filesystem::path output_directory_;
  std::ofstream poses_file_;
  double seconds_per_node_ = 1.0;
  double meters_per_node_ = 0.05;
  double last_saved_seconds_ = 0.0;
  double last_saved_x_ = 0.0;
  double last_saved_y_ = 0.0;
  int saved_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TopomapRecorderNode>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "topomap_recorder_node: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
