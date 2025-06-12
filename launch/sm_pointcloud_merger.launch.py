from launch import LaunchDescription
from launch_ros.actions import Node
import yaml, os

def generate_launch_description():
    config = {
        'topics': [
            '/lidar_front/points',
            '/lidar_left/points',
            '/lidar_right/points'
        ],
        'output_topic': '/merged_points'
    }

    node = Node(
        package='sm_pointcloud_merger_ros2',
        executable='pointcloud_merger_node',
        name='pointcloud_merger',
        parameters=[config],
        output='screen'
    )

    return LaunchDescription([node])
