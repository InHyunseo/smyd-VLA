// topomap_recorder_node
// 사람이 로봇을 몰고 다니는 동안 카메라 영상을 일정 간격으로 저장하고 그때의 odom 위치를 함께 적는다.
// 저장한 폴더가 ViNT의 topological map이 되고, 마지막 줄의 위치가 주행 평가의 목표가 된다.
//
// 입력: output_directory, seconds_per_node 파라미터, camera/image_raw (Image), odom (Odometry)
// 출력: output_directory/NNNN.png, output_directory/poses.csv

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
    if (seconds_per_node_ <= 0.0) {throw std::runtime_error("seconds_per_node must be positive");}
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

  // 지금까지 저장한 노드 수.
  int saved_count() const {return saved_count_;}

private:
  // 간격이 지났고 odom을 받은 상태면 노드 하나를 저장한다.
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    const double seconds = rclcpp::Time(message->header.stamp).seconds();
    if (last_odometry_ == nullptr || (saved_count_ > 0 && seconds - last_saved_seconds_ < seconds_per_node_)) {
      return;
    }
    char name[16];
    std::snprintf(name, sizeof(name), "%04d.png", saved_count_);
    if (!cv::imwrite(
        (output_directory_ / name).string(), cv_bridge::toCvShare(message, "bgr8")->image))
    {
      throw std::runtime_error("cannot save topomap image");
    }

    const auto & position = last_odometry_->pose.pose.position;
    poses_file_ << saved_count_ << "," << position.x << "," << position.y << ","
                << tf2::getYaw(last_odometry_->pose.pose.orientation) << "," << seconds << "\n";
    poses_file_.flush();          // Ctrl+C로 끝내도 기록이 남도록
    last_saved_seconds_ = seconds;
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
  double last_saved_seconds_ = 0.0;
  int saved_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::shared_ptr<TopomapRecorderNode> node = std::make_shared<TopomapRecorderNode>();
  rclcpp::spin(node);
  RCLCPP_INFO(node->get_logger(), "saved %d topomap nodes", node->saved_count());
  rclcpp::shutdown();
  return 0;
}
