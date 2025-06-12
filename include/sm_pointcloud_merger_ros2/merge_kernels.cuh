#pragma once
#include <vector>
#include <sensor_msgs/msg/point_cloud2.hpp>

// 호스트 함수: ROS2 메시지 벡터를 GPU에서 병합해 새 메시지 작성
void gpu_merge_pointclouds(
  const std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr>& input_clouds,
  sensor_msgs::msg::PointCloud2& output);
