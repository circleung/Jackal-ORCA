"""
lidar_detector.launch.py — Phase 1.2
Livox Mid-360 점군 클러스터링 단독 실행
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jackal_orca_perception',
            executable='lidar_obstacle_detector.py',
            name='lidar_obstacle_detector',
            parameters=[{
                'input_topic': '/livox/lidar',
                'range_min_m': 0.3,
                'range_max_m': 10.0,
                'height_min_m': 0.1,
                'height_max_m': 2.0,
                'voxel_size_m': 0.05,
                'dbscan_eps_m': 0.30,
                'dbscan_min_samples': 10,
                'min_cluster_points': 30,
            }],
            output='screen',
        ),
    ])
