#include "sm_pointcloud_merger_ros2/merge_kernels.cuh"
#include <cuda_runtime.h>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <thrust/device_vector.h>
#include <thrust/copy.h>

using sensor_msgs::msg::PointCloud2;

namespace
{
  // ────────────────────────────────────────────────────────────────────
  // CUDA 커널: 4×4 변환행렬을 적용하여 src → dst 복사
  // (행렬은 행 우선, 12원소 배열)
  // ────────────────────────────────────────────────────────────────────
  __global__ void transform_and_copy_kernel(
    const float4* __restrict__ src,
    float4* __restrict__ dst,
    const float* __restrict__ tf, int N)
  {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < N)
    {
      float4 p = src[i];
      float x = tf[0]*p.x + tf[1]*p.y + tf[2] *p.z + tf[3];
      float y = tf[4]*p.x + tf[5]*p.y + tf[6] *p.z + tf[7];
      float z = tf[8]*p.x + tf[9]*p.y + tf[10]*p.z + tf[11];
      dst[i] = make_float4(x, y, z, p.w);
    }
  }

  // 디바이스에 미리 올려둘 단순 4×4 단위 행렬
  __constant__ float IDENTITY_TF[12] = {1,0,0,0, 0,1,0,0, 0,0,1,0};

  // ────────────────────────────────────────────────────────────────────
  // 헬퍼: PCL→디바이스 float4 포인터 변환
  // ────────────────────────────────────────────────────────────────────
    struct DeviceCloud
    {
        thrust::device_vector<float4> data;
        int size() const { return static_cast<int>(data.size()); }
        float4* ptr()     { return thrust::raw_pointer_cast(data.data()); }
    };

    DeviceCloud to_device(const pcl::PointCloud<pcl::PointXYZI>& cloud)
    {
        // 1) 호스트에서 float4 배열 생성
        std::vector<float4> host_data;
        host_data.reserve(cloud.points.size());
        for (const auto &p : cloud.points) {
            float4 v;
            v.x = p.x; v.y = p.y; v.z = p.z; v.w = p.intensity;
            host_data.push_back(v);
        }

        // 2) 디바이스 벡터에 업로드
        DeviceCloud dev;
        dev.data = thrust::device_vector<float4>(
        host_data.begin(), host_data.end());
        return dev;
    }
}

// ──────────────────────────────────────────────────────────────────────
// 호스트 함수 구현
// ──────────────────────────────────────────────────────────────────────
void gpu_merge_pointclouds(
  const std::vector<PointCloud2::ConstSharedPtr>& input_clouds,
  PointCloud2& output)
{
  // 1. PointCloud2 → PCL 변환
  std::vector<pcl::PointCloud<pcl::PointXYZI>> pcl_vec(input_clouds.size());
  size_t total_points = 0;
  for (size_t i=0; i<input_clouds.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZI> tmp;
    pcl::fromROSMsg(*input_clouds[i], tmp);
    total_points += tmp.size();
    pcl_vec[i] = std::move(tmp);
  }

  // 2. 디바이스 메모리 복사
  std::vector<DeviceCloud> dev_vec;
  dev_vec.reserve(pcl_vec.size());
  for (auto & pc : pcl_vec) dev_vec.emplace_back(to_device(pc));

  // 3. 결과 버퍼 생성
  thrust::device_vector<float4> merged_dev(total_points);

  // 4. 각 LiDAR 변환행렬(예: 단위행렬) 적용 및 큰 버퍼에 복사
  size_t offset = 0;
  const int threads = 256;
  for (auto & dc : dev_vec)
  {
    int blocks = (dc.size() + threads - 1) / threads;
    transform_and_copy_kernel<<<blocks, threads>>>(
      dc.ptr(),
      thrust::raw_pointer_cast(merged_dev.data()) + offset,
      IDENTITY_TF, dc.size());
    offset += dc.size();
  }
  cudaDeviceSynchronize();

  // 5. 디바이스→호스트 float4 배열 복사
  std::vector<float4> host_data(total_points);
  thrust::copy(merged_dev.begin(), merged_dev.end(), host_data.begin());

  // 6. float4 → PCL PointXYZI 변환
  pcl::PointCloud<pcl::PointXYZI> merged_host;
  merged_host.points.resize(total_points);
  for (size_t i = 0; i < total_points; ++i) {
      const float4 &v = host_data[i];
      pcl::PointXYZI p;
      p.x = v.x; p.y = v.y; p.z = v.z; p.intensity = v.w;
      merged_host.points[i] = p;
  }
  // 7. PCL → PointCloud2 메시지로 변환 및 헤더 복사
  pcl::toROSMsg(merged_host, output);
  output.header = input_clouds.front()->header;
}
