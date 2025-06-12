from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = {
        'topics': [
            '/lidar_front/points',
            '/lidar_left/points',
            '/lidar_right/points',
            '/lidar_rear/points'
        ],
        'output_topic': '/merged_points',
        'output_frame_id': 'odom',
        'voxel_leaf_size': 0.1
    }

    return LaunchDescription([
        Node(
            package='sm_pointcloud_merger_ros2',
            executable='pointcloud_merger_node',
            name='pointcloud_merger',
            parameters=[params],
            output='screen'
        )
    ])