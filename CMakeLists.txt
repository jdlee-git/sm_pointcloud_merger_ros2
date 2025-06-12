cmake_minimum_required(VERSION 3.22)
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
project(sm_pointcloud_merger_ros2 LANGUAGES C CXX CUDA)  # C 언어 활성화

# ── 빌드 옵션 ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ── ROS 2 의존성 ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp          REQUIRED)
find_package(sensor_msgs     REQUIRED)
find_package(tf2_ros         REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros         REQUIRED)
find_package(MPI REQUIRED COMPONENTS C)

# ── PCL + CUDA ───────────────────────────────────────────────────────────
find_package(PCL            REQUIRED COMPONENTS common io)
find_package(CUDAToolkit 12.4 REQUIRED)

add_compile_definitions(PCL_NO_PRECOMPILE)

include_directories(
  ${PCL_INCLUDE_DIRS}
  include
)

# ── 실행 파일 ────────────────────────────────────────────────────────────
add_executable(pointcloud_merger_node
  src/pointcloud_merger_node.cpp
  src/merge_kernels.cu
)

target_link_libraries(pointcloud_merger_node
  ${PCL_LIBRARIES}
  CUDA::cudart
)

ament_target_dependencies(pointcloud_merger_node
  rclcpp sensor_msgs pcl_conversions pcl_ros
)

# ── 설치 ─────────────────────────────────────────────────────────────────
install(TARGETS pointcloud_merger_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
