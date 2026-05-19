"""
depth_detector.launch.py — Phase 1.1
camera1+camera2 depth 검출 + 자칼 Mini PC 사운드 자동 시작
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


# Mini PC 사운드 연동 설정
MINI_PC_USER = 'jackal'
MINI_PC_IP = '192.168.55.100'
AUDIO_SCRIPT = '/home/jackal/colcon_ws/src/jackal_audio/scripts/audio_player_node.py'


def generate_launch_description():
    common_params = {
        'min_distance_m': 0.3,
        'max_distance_m': 5.0,
        'min_area_px': 200,
        'warn_dist_m': 1.5,
        'danger_dist_m': 0.8,
        'morph_kernel': 5,
        'floor_crop_ratio': 0.55,
    }

    nodes = [
        # camera1 detector
        Node(
            package='jackal_orca_perception',
            executable='depth_object_detector.py',
            name='depth_detector_camera1',
            parameters=[{**common_params, 'camera_name': 'camera1'}],
            output='screen',
        ),
        # camera2 detector
        Node(
            package='jackal_orca_perception',
            executable='depth_object_detector.py',
            name='depth_detector_camera2',
            parameters=[{**common_params, 'camera_name': 'camera2'}],
            output='screen',
        ),
        # /perception/depth_active 토픽으로 1Hz Bool true publish (heartbeat)
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--rate', '1',
                 '/perception/depth_active',
                 'std_msgs/msg/Bool', 'data: true'],
            output='screen',
            name='depth_active_heartbeat',
        ),
        # Mini PC에 SSH로 audio_player_node 자동 실행
        ExecuteProcess(
            cmd=['ssh',
                 '-o', 'StrictHostKeyChecking=no',
                 '-o', 'ServerAliveInterval=30',
                 f'{MINI_PC_USER}@{MINI_PC_IP}',
                 f'source /opt/ros/jazzy/setup.bash && '
                 f'export ROS_DOMAIN_ID=0 && '
                 f'python3 {AUDIO_SCRIPT}'],
            output='screen',
            name='minipc_audio_player',
        ),
    ]

    return LaunchDescription(nodes)
