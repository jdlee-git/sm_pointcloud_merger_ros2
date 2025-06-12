#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.hpp>
#include "sm_pointcloud_merger_ros2/merge_kernels.cuh"

using sensor_msgs::msg::PointCloud2;

class PointCloudMergerNode : public rclcpp::Node
{
public:
  PointCloudMergerNode()
  : Node("pointcloud_merger_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // 파라미터
    declare_parameter<std::vector<std::string>>("topics", {"/lidar_front/points","/lidar_left/points","/lidar_right/points","/lidar_rear/points"});
    declare_parameter<std::string>("output_topic", "/merged_points");
    declare_parameter<std::string>("output_frame_id", "base_link");
    declare_parameter<double>("voxel_leaf_size", 0.05);

    // 읽어오기
    topics_ = get_parameter("topics").as_string_array();
    output_topic_ = get_parameter("output_topic").as_string();
    output_frame_id_ = get_parameter("output_frame_id").as_string();
    voxel_leaf_size_ = get_parameter("voxel_leaf_size").as_double();

    latest_.resize(topics_.size());

    // 구독자 생성
    for(size_t i=0; i<topics_.size(); ++i) {
      subs_.push_back(create_subscription<PointCloud2>(
        topics_[i], rclcpp::SensorDataQoS(),
        [this, i](PointCloud2::ConstSharedPtr msg) {
          this->cloud_callback(msg, i);
        }));
    }

    // 퍼블리셔
    pub_ = create_publisher<PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
  }

private:
  void cloud_callback(PointCloud2::ConstSharedPtr msg, size_t idx)
  {
    // 1) TF 변환: 로컬 프레임 -> output_frame_id_
    PointCloud2 transformed;
    try {
      auto t = tf_buffer_.lookupTransform(
        output_frame_id_, msg->header.frame_id,
        msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
      pcl_ros::transformPointCloud(
        output_frame_id_, t, *msg, transformed);
    } catch (...) {
      RCLCPP_WARN(get_logger(), "TF lookup failed for %s", msg->header.frame_id.c_str());
      return;
    }

    latest_[idx] = std::make_shared<PointCloud2>(transformed);

    // 모두 수집되었으면 GPU merge 호출
    if (std::all_of(latest_.begin(), latest_.end(), [](auto &p){ return p!=nullptr; })) {
      PointCloud2 merged;
      gpu_merge_pointclouds(latest_, merged);

      // 2) VoxelGrid 필터링
      pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
      pcl::fromROSMsg(merged, pcl_cloud);
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(pcl_cloud.makeShared());
      vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      pcl::PointCloud<pcl::PointXYZI> filtered;
      vg.filter(filtered);
      pcl::toROSMsg(filtered, merged);

      // 3) 헤더 설정 후 퍼블리시
      merged.header.frame_id = output_frame_id_;
      merged.header.stamp = latest_.front()->header.stamp;
      pub_->publish(merged);
    }
  }

  std::vector<std::string> topics_;
  std::vector<PointCloud2::ConstSharedPtr> latest_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> subs_;
  std::string output_topic_, output_frame_id_;
  double voxel_leaf_size_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudMergerNode>());
  rclcpp::shutdown();
  return 0;
}