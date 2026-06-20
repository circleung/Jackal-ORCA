"""
perception.launch.py — 젯슨 인식 ↔ 우리 SLAM 융합 (태그 수집 + 클러스터링).

띄우는 것:
  1. base_link → camera_{front,back}_link static TF   (마운트 위치 — ★줄자 실측 보정)
  2. camera_{front,back}_link → ..._color_optical_frame static TF (REP-103 optical 회전, 고정)
  3. tag_collector : apriltag 검출 → solvePnP → map 누적 → /tags_in_map + tag_observations.json
  4. clustering   : /tags_in_map → DBSCAN → /hotspots + /hotspot_markers

젯슨은 TF 를 일절 발행하지 않으므로 카메라 TF 는 mini PC(여기)에서 발행해야
apriltag detection(optical frame) → map 변환이 성립한다.

선행 조건:
  1. 젯슨 인식 파이프라인 실행 중 (/apriltag_*/detections, /camera_*/color/camera_info)
  2. slam_2d.launch.py (/map + map→odom TF) + Clearpath bringup (odom→base_link)

사용:
  ros2 launch tag_hotspot_nav perception.launch.py
  ros2 launch tag_hotspot_nav perception.launch.py tag_size:=0.137 cam_front_z:=0.28

좌표 규약(REP-103):
  - *_link : x=정면, y=좌, z=상 (마운트는 이 프레임 기준으로 줄자 측정)
  - *_optical_frame : z=렌즈 정면, x=우, y=하 (← link 에서 rpy=-π/2,0,-π/2 회전, 고정값)
"""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# camera_link → camera_optical_frame 고정 회전 (절대 건드리지 말 것)
OPTICAL_RPY = ['--roll', str(-math.pi / 2), '--pitch', '0', '--yaw', str(-math.pi / 2)]


def static_tf(name, parent, child, x='0', y='0', z='0',
              roll='0', pitch='0', yaw='0', tf_remaps=None):
    args = ['--x', x, '--y', y, '--z', z,
            '--roll', roll, '--pitch', pitch, '--yaw', yaw,
            '--frame-id', parent, '--child-frame-id', child]
    return Node(package='tf2_ros', executable='static_transform_publisher',
                name=name, arguments=args, remappings=tf_remaps or [])


def optical_tf(name, parent, child, tf_remaps=None):
    args = ['--x', '0', '--y', '0', '--z', '0'] + OPTICAL_RPY + \
           ['--frame-id', parent, '--child-frame-id', child]
    return Node(package='tf2_ros', executable='static_transform_publisher',
                name=name, arguments=args, remappings=tf_remaps or [])


def generate_launch_description():
    # slam_2d/explore 와 동일 — Clearpath 가 /j100_0915/tf 로 발행하므로 리매핑 필수
    tf_remaps = [
        ('/tf',        '/j100_0915/tf'),
        ('/tf_static', '/j100_0915/tf_static'),
    ]

    tag_size = LaunchConfiguration('tag_size')
    map_frame = LaunchConfiguration('map_frame')

    # ── 카메라 마운트 placeholder (★줄자로 실측해서 보정할 것) ──────────
    #   base_link 원점(차체 중심, 지면 아님) 기준. front 는 정면, back 은 yaw=π.
    fx = LaunchConfiguration('cam_front_x')
    fz = LaunchConfiguration('cam_front_z')
    bx = LaunchConfiguration('cam_back_x')
    bz = LaunchConfiguration('cam_back_z')

    return LaunchDescription([
        DeclareLaunchArgument('tag_size', default_value='0.15',
                              description='태그 검은 사각형 변 길이 [m] (흰 여백 제외, apriltag.yaml 의 size 와 일치)'),
        DeclareLaunchArgument('map_frame', default_value='map',
                              description='태그 누적 기준 프레임. slam 없이 검증 시 base_link 로'),
        DeclareLaunchArgument('cam_front_x', default_value='0.20',
                              description='[placeholder] base_link→front 카메라 전방 오프셋 [m]'),
        DeclareLaunchArgument('cam_front_z', default_value='0.30',
                              description='[placeholder] front 카메라 높이 [m]'),
        DeclareLaunchArgument('cam_back_x', default_value='-0.20',
                              description='[placeholder] base_link→back 카메라 후방 오프셋 [m]'),
        DeclareLaunchArgument('cam_back_z', default_value='0.30',
                              description='[placeholder] back 카메라 높이 [m]'),

        # 1) base_link → camera_*_link (마운트 — 실측 보정 대상)
        static_tf('base_to_cam_front', 'base_link', 'camera_front_link',
                  x=fx, z=fz, yaw='0', tf_remaps=tf_remaps),
        static_tf('base_to_cam_back', 'base_link', 'camera_back_link',
                  x=bx, z=bz, yaw=str(math.pi), tf_remaps=tf_remaps),

        # 2) camera_*_link → optical_frame (REP-103 고정 회전)
        optical_tf('cam_front_optical', 'camera_front_link',
                   'camera_front_color_optical_frame', tf_remaps=tf_remaps),
        optical_tf('cam_back_optical', 'camera_back_link',
                   'camera_back_color_optical_frame', tf_remaps=tf_remaps),

        # 3) tag_collector
        Node(
            package='tag_hotspot_nav',
            executable='tag_collector',
            name='tag_collector',
            output='screen',
            parameters=[{'tag_size': tag_size, 'map_frame': map_frame}],
            remappings=tf_remaps,
        ),
        # 4) clustering — 누적 태그 DBSCAN 군집화 → /hotspots (Phase 3)
        Node(
            package='tag_hotspot_nav',
            executable='clustering',
            name='clustering',
            output='screen',
            parameters=[{
                'eps': 2.0,          # [m] 같은 hotspot으로 묶을 태그 간 최대 거리
                'min_samples': 2,    # hotspot 성립 최소 태그 수(자기 포함)
                'cluster_period': 3.0,
                'map_frame': map_frame,
            }],
        ),
    ])
