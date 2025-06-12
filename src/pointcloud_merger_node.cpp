#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sm_pointcloud_merger_ros2/merge_kernels.cuh"

using sensor_msgs::msg::PointCloud2;

class PointCloudMergerNode : public rclcpp::Node
{
public:
  PointCloudMergerNode()
  : Node("pointcloud_merger_node")
  {
    // 파라미터 선언
    declare_parameter<std::vector<std::string>>("topics", {
      "/lidar_front/points",
      "/lidar_left/points",
      "/lidar_right/points"
    });
    declare_parameter<std::string>("output_topic", "/merged_points");
    declare_parameter<std::string>("output_frame_id", "base_link");  // 추가

    // 파라미터 읽기
    auto topic_names = get_parameter("topics").as_string_array();
    output_topic_   = get_parameter("output_topic").as_string();
    output_frame_id_ = get_parameter("output_frame_id").as_string();  // 읽어두기

    latest_.resize(topic_names.size());

    // 구독자 생성 (람다 사용)
    for (size_t i = 0; i < topic_names.size(); ++i) {
      const size_t idx = i;
      subs_.push_back(create_subscription<PointCloud2>(
        topic_names[i], rclcpp::SensorDataQoS(),
        [this, idx](PointCloud2::ConstSharedPtr msg) {
          this->cloud_callback(msg, idx);
        }));
    }

    // 퍼블리셔
    pub_ = create_publisher<PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
  }

private:
  void cloud_callback(PointCloud2::ConstSharedPtr msg, size_t idx)
  {
    latest_[idx] = msg;
    if (std::all_of(latest_.begin(), latest_.end(),
      [](auto &p){ return p != nullptr; }))
    {
      PointCloud2 merged;
      gpu_merge_pointclouds(latest_, merged);
      // 헤더 설정: 파라미터로 받은 frame_id & 타임스탬프
      merged.header.frame_id = output_frame_id_;
      merged.header.stamp    = latest_.front()->header.stamp;
      pub_->publish(merged);
    }
  }

  std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> subs_;
  std::vector<PointCloud2::ConstSharedPtr> latest_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  std::string output_topic_;
  std::string output_frame_id_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudMergerNode>());
  rclcpp::shutdown();
  return 0;
}
