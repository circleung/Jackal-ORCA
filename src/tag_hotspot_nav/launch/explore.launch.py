"""
explore.launch.py — 자율 탐사 스택 (frontier_explorer + pure_pursuit [+ safety_layer] [+ sound]).

선행 조건:
  1. roas2_bringup platform.launch.py  (휠오돔 TF + twist_mux + 모터)
  2. tag_hotspot_nav slam_2d.launch.py (/map + map→odom TF + /scan)

사용:
  ros2 launch tag_hotspot_nav explore.launch.py
  ros2 launch tag_hotspot_nav explore.launch.py linear_speed:=0.3     # 빠른 모드
  ros2 launch tag_hotspot_nav explore.launch.py cmd_vel_topic:=/dry_run/cmd_vel  # 바퀴 안 굴림
  ros2 launch tag_hotspot_nav explore.launch.py use_safety:=false     # 안전 게이트 끄기
  ros2 launch tag_hotspot_nav explore.launch.py use_sound:=false      # 사운드 끄기

⚠ use_safety:=true(기본): 체인이 pure_pursuit→/cmd_vel_raw→safety_layer→cmd_vel.
   정지거리 front/back_stop_dist=0.50m (base_link 기준). pure_pursuit stop_dist=0.60m
   와 이중 보호. 유리창 근접정지의 2차 안전망.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    linear_speed = LaunchConfiguration('linear_speed')
    use_safety = LaunchConfiguration('use_safety')
    use_sound = LaunchConfiguration('use_sound')
    use_map_cleaner = LaunchConfiguration('use_map_cleaner')
    use_tag_centering = LaunchConfiguration('use_tag_centering')
    dynamic_clean = LaunchConfiguration('dynamic_clean')

    # map_cleaner 켜면 frontier 는 잔상 제거된 /map_nav 를 본다
    map_topic = PythonExpression(
        ["'/map_nav' if '", use_map_cleaner, "' == 'true' else '/map'"])

    # Clearpath bringup 이 TF 를 /j100_0915/tf(_static) 로 발행 → TF 쓰는 노드는 리매핑 필요
    tf_remaps = [
        ('/tf',        '/j100_0915/tf'),
        ('/tf_static', '/j100_0915/tf_static'),
    ]

    # use_safety 면 pure_pursuit 출력을 /cmd_vel_raw 로 돌려 safety_layer 를 거치게 한다.
    pp_cmd_topic = PythonExpression(
        ["'/cmd_vel_raw' if '", use_safety, "' == 'true' else '", cmd_vel_topic, "'"])

    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/j100_0915/cmd_vel',
                              description='최종 주행 명령 토픽 (twist_mux 입력)'),
        DeclareLaunchArgument('linear_speed', default_value='0.3',
                              description='최대 선속도 [m/s] — 유리창 대비 SLAM 선행 시간 확보'),
        DeclareLaunchArgument('use_safety', default_value='true',
                              description='cmd_vel 안전 게이트(safety_layer) 삽입 — 유리창 근접정지 2차 보호'),
        DeclareLaunchArgument('use_sound', default_value='true',
                              description='이벤트 사운드(sound_player) 실행'),
        DeclareLaunchArgument('use_map_cleaner', default_value='true',
                              description='동적장애물 잔상 제거(map_cleaner) → frontier 가 /map_nav 사용'),
        DeclareLaunchArgument('use_tag_centering', default_value='true',
                              description='매핑 중 태그 후보에 정렬해 또렷이 포착(tag_centering)'),
        DeclareLaunchArgument('dynamic_clean', default_value='false',
                              description='map_cleaner 동적 clear+/scan 장애물 마킹(off면 slam맵+keep-out만)'),

        Node(
            package='tag_hotspot_nav',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[{
                'robot_radius': 0.20,
                'goal_timeout': 100.0,
                'no_frontier_limit': 5,
                'heading_weight': 4.0,
                'scan_spin_duration': 0.0,
            }],
            remappings=tf_remaps + [('/map', map_topic)],
        ),
        Node(
            package='tag_hotspot_nav',
            executable='pure_pursuit',
            name='pure_pursuit',
            output='screen',
            parameters=[{
                'cmd_vel_topic': pp_cmd_topic,
                'linear_speed': linear_speed,
                'lookahead': 0.7,
                'max_angular': 0.8,
                'goal_tolerance': 0.30,
                'stop_dist': 0.55,
                'slow_down_dist': 1.00,
                'front_sector_deg': 35.0,
            }],
            remappings=tf_remaps,
        ),

        # 안전 게이트 (기본 on) — /cmd_vel_raw → cmd_vel_topic
        Node(
            package='tag_hotspot_nav',
            executable='safety_layer',
            name='safety_layer',
            output='screen',
            condition=IfCondition(use_safety),
            parameters=[{
                'raw_topic': '/cmd_vel_raw',
                'cmd_vel_topic': cmd_vel_topic,
                'scan_topic': '/scan',
                'max_linear': linear_speed,
                'front_stop_dist': 0.50,
                'back_stop_dist': 0.50,
                'swept_half_width': 0.235,
            }],
        ),

        # 이벤트 사운드 (기본 on)
        Node(
            package='tag_hotspot_nav',
            executable='sound_player',
            name='sound_player',
            output='screen',
            condition=IfCondition(use_sound),
        ),

        # 끼임 감지 (명령 지속 + 휠오돔 무이동 → /stuck)
        Node(
            package='tag_hotspot_nav',
            executable='stuck_detector',
            name='stuck_detector',
            output='screen',
        ),

        # 매핑 중 태그 정렬
        Node(
            package='tag_hotspot_nav',
            executable='tag_centering',
            name='tag_centering',
            output='screen',
            condition=IfCondition(use_tag_centering),
            parameters=[{'cmd_vel_topic': pp_cmd_topic}],
            remappings=tf_remaps,
        ),

        # 동적장애물 잔상 제거
        Node(
            package='tag_hotspot_nav',
            executable='map_cleaner',
            name='map_cleaner',
            output='screen',
            condition=IfCondition(use_map_cleaner),
            parameters=[{'dynamic_clean': dynamic_clean}],
            remappings=tf_remaps,
        ),

        # PS4 X버튼 비상정지
        Node(
            package='tag_hotspot_nav',
            executable='joy_estop',
            name='joy_estop',
            output='screen',
        ),
    ])
