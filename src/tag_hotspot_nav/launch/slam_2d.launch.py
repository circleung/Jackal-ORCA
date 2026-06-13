"""
slam_2d.launch.py — 정밀 2D SLAM 파이프라인 (실기체 Jackal + MID360)

띄우는 것:
  1. livox_ros_driver2  : MID360 → /livox/lidar (PointCloud2, xfer_format=0)
                          ※ FAST-LIO용 CustomMsg 드라이버와 동시 실행 불가(같은 장치 점유)
  2. static TF          : base_link → livox_frame (마운트 위치, 실측값으로 보정할 것)
  3. pointcloud_to_laserscan : 3D 클라우드 → /scan (z 0.2~0.6m 벽 슬라이스)
  4. slam_toolbox(online_async) : /scan + 휠오돔 TF → /map + map→odom TF

선행 조건:
  - roas2_bringup platform.launch.py 실행 중 (odom→base_link TF 제공)

사용:
  ros2 launch tag_hotspot_nav slam_2d.launch.py
  ros2 launch tag_hotspot_nav slam_2d.launch.py use_livox:=false   # 드라이버 별도 실행 시
  ros2 launch tag_hotspot_nav slam_2d.launch.py lidar_z:=0.42      # 마운트 실측 반영
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_share   = get_package_share_directory('tag_hotspot_nav')
    livox_share = get_package_share_directory('livox_ros_driver2')

    # ── TF 토픽 리매핑 ──────────────────────────────────────────────
    # Clearpath bringup 이 TF 를 /j100_0915/tf(_static) 네임스페이스로 발행
    # (프레임 이름 자체는 odom, base_link 로 prefix 없음).
    # 우리 노드들은 기본적으로 /tf 를 보므로 전부 리매핑해야
    # slam_toolbox 가 odom→base_link 를 찾을 수 있다.
    tf_remaps = [
        ('/tf',        '/j100_0915/tf'),
        ('/tf_static', '/j100_0915/tf_static'),
    ]

    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    p2l_params  = os.path.join(pkg_share, 'config', 'pointcloud_to_laserscan_params.yaml')
    livox_cfg   = os.path.join(livox_share, 'config', 'MID360_config.json')

    use_livox = LaunchConfiguration('use_livox')
    # ── base_link → livox_frame 마운트 오프셋 ──────────────────────
    # z 기본값 0.39 는 기존 octomap 설정의 "LIDAR_H=0.39m(지면 기준)" 주석에서 가져옴.
    # base_link 원점 높이에 따라 달라지므로 반드시 줄자로 실측해서 보정할 것.
    # yaw 가 틀어지면 스캔매칭-휠오돔이 서로 싸워 맵이 휜다 → 커넥터 방향 확인.
    lidar_x   = LaunchConfiguration('lidar_x')
    lidar_y   = LaunchConfiguration('lidar_y')
    lidar_z   = LaunchConfiguration('lidar_z')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    # ── slam_toolbox (Jazzy 2.8.x 는 lifecycle 노드) ───────────────
    # 그냥 띄우면 unconfigured 로 멈춰 /scan 구독을 안 하므로,
    # 공식 online_async_launch.py 와 같은 방식으로
    # 시작 직후 configure → (inactive 도달 시) activate 를 자동 발행한다.
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params],
        remappings=tf_remaps,
    )
    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring', goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_livox', default_value='true',
                              description='livox 드라이버를 이 launch 에서 직접 실행'),
        # 실측 (2026-06-08): 로봇 길이 52cm(중심→앞단 26cm), 라이더는 앞단 14cm 뒤 → x=+0.12.
        # 라이더 지면 43cm − base_link 지면높이 0.0635m(=wheel_radius0.098−wheel_vertical_offset0.0345) → z=0.367
        DeclareLaunchArgument('lidar_x',   default_value='0.12'),
        DeclareLaunchArgument('lidar_y',   default_value='0.0'),
        DeclareLaunchArgument('lidar_z',   default_value='0.367'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),

        # 1) MID360 드라이버 — PointCloud2 모드
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            condition=IfCondition(use_livox),
            parameters=[{
                'xfer_format':   0,        # 0 = sensor_msgs/PointCloud2
                'multi_topic':   0,
                'data_src':      0,
                'publish_freq':  10.0,
                'output_data_type': 0,
                'frame_id':      'livox_frame',
                'user_config_path': livox_cfg,
            }],
        ),

        # 2) base_link → livox_frame static TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_livox_static',
            arguments=['--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
                       '--yaw', lidar_yaw, '--pitch', '0', '--roll', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
            remappings=tf_remaps,
        ),

        # 3) 3D → 2D 변환
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[p2l_params],
            remappings=[
                ('cloud_in', '/livox/lidar'),
                ('scan',     '/scan'),
            ] + tf_remaps,
        ),

        # 4) slam_toolbox (online_async) — lifecycle 자동 기동
        slam_node,
        slam_configure,
        slam_activate,

        # 5) 탐사 전용 Foxglove bridge (포트 8766)
        #    Clearpath 기본 bridge(8765)와 별개 — ws://<로봇IP>:8766 으로 접속
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge_nav',
            output='screen',
            parameters=[{'port': 8766}],
        ),

        # 6) 자칼 부피 시각화 — base_link 에 박스 Marker(/jackal_volume).
        #    URDF(robot_description) 는 package:// 메시라 8766 브리지가 못 가져와
        #    Foxglove 에서 link 에러가 남 → 메시 무관한 박스로 부피 표시.
        Node(
            package='tag_hotspot_nav',
            executable='footprint_marker',
            name='footprint_marker',
            output='screen',
        ),
    ])
