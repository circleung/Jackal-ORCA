"""
Jackal-ORCA — 전체 센서 bringup (USB 대역폭 최적화)
- RealSense D435i × 2 (424x240x15, color+depth only)
- Livox Mid-360 (LiDAR + IMU)
- TF + RViz(옵션)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


CAM1_SERIAL = "344522070202"
CAM2_SERIAL = "344522070059"
CAM_PROFILE = "424x240x15"


def realsense_node(name, serial):
    return Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=name,
        parameters=[{
            'serial_no': serial,
            'camera_name': name,
            'camera_namespace': '',

            # 스트림 활성/비활성
            'enable_color': True,
            'enable_depth': True,
            'enable_infra': False,
            'enable_infra1': False,            # ★ 추가 — IR1 명시적 차단
            'enable_infra2': False,            # ★ 추가 — IR2 명시적 차단
            'enable_gyro': False,
            'enable_accel': False,
            'pointcloud.enable': False,
            'align_depth.enable': False,

            # 해상도/FPS — depth와 color 둘 다
            'depth_module.depth_profile': CAM_PROFILE,
            'rgb_camera.color_profile': CAM_PROFILE,   # ★ 핵심 — color는 이 이름 써야 함
        }],
        output='screen',
    )


def livox_node():
    livox_share = get_package_share_directory('livox_ros_driver2')
    cfg = os.path.join(livox_share, 'config', 'MID360_config.json')
    return Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 0,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': cfg,
        }],
    )


def static_tf(name, parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
            '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
            '--frame-id', parent, '--child-frame-id', child,
        ],
    )


def rviz_node():
    bringup_share = get_package_share_directory('jackal_orca_bringup')
    rviz_cfg = os.path.join(bringup_share, 'rviz', 'bringup_all.rviz')
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
    )


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),

        LogInfo(msg=f"[bringup_all] CAM1={CAM1_SERIAL}, CAM2={CAM2_SERIAL}, profile={CAM_PROFILE}"),

        realsense_node('camera1', CAM1_SERIAL),
        TimerAction(period=5.0, actions=[realsense_node('camera2', CAM2_SERIAL)]),

        livox_node(),

        static_tf('base_to_livox', 'base_link', 'livox_frame', xyz=(0.10, 0.0, 0.20)),
        static_tf('base_to_cam1', 'base_link', 'camera1_link', xyz=(0.15, 0.0, 0.10)),
        static_tf('base_to_cam2', 'base_link', 'camera2_link', xyz=(-0.15, 0.0, 0.10), rpy=(0, 0, 3.14159)),

        TimerAction(period=3.0, actions=[rviz_node()], condition=IfCondition(use_rviz)),
    ])
