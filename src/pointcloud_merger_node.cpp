#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sm_pointcloud_merger_ros2/merge_kernels.cuh"

using std::placeholders::_1;
using sensor_msgs::msg::PointCloud2;

class PointCloudMergerNode : public rclcpp::Node
{
public:
  PointCloudMergerNode()
  : Node("pointcloud_merger_node")
  {
    // 파라미터 선언
    declare_parameter<std::vector<std::string>>("topics",
      {"/lidar_front/points", "/lidar_left/points", "/lidar_right/points"});
    declare_parameter<std::string>("output_topic", "/merged_points");

    auto topic_names = get_parameter("topics").as_string_array();
    output_topic_   = get_parameter("output_topic").as_string();

    latest_.resize(topic_names.size());

    // 구독자 생성
    for (size_t i = 0; i < topic_names.size(); ++i)
    {
        const size_t idx = i;
        subs_.push_back(this->create_subscription<PointCloud2>(
        topic_names[i], rclcpp::SensorDataQoS(),
        [this, idx](const PointCloud2::ConstSharedPtr msg) {
        this->cloud_callback(msg, idx);
        }));
    }

    // 퍼블리셔 생성
    pub_ = create_publisher<PointCloud2>(output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
      "Subscribing %zu LiDAR topics, publishing merged cloud on %s",
      topic_names.size(), output_topic_.c_str());
  }

private:
  void cloud_callback(const PointCloud2::ConstSharedPtr msg, size_t idx)
  {
    // 최신 메시지 캐시
    latest_[idx] = msg;

    // 모두 도착했는지 확인
    if (std::all_of(latest_.begin(), latest_.end(),
                    [](auto & p){ return p != nullptr; }))
    {
      PointCloud2 merged;
      gpu_merge_pointclouds(latest_, merged);
      pub_->publish(merged);
    }
  }

  // 멤버 변수
  std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> subs_;
  std::vector<PointCloud2::ConstSharedPtr> latest_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  std::string output_topic_;
};

// ────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudMergerNode>());
  rclcpp::shutdown();
  return 0;
}
