"""
explore.launch.py — 자율 탐사 스택 (frontier_explorer + pure_pursuit [+ safety_layer] [+ sound]).

선행 조건:
  1. roas2_bringup platform.launch.py  (휠오돔 TF + twist_mux + 모터)
  2. tag_hotspot_nav slam_2d.launch.py (/map + map→odom TF + /scan)

사용:
  ros2 launch tag_hotspot_nav explore.launch.py
  ros2 launch tag_hotspot_nav explore.launch.py linear_speed:=0.2     # 조심 모드
  ros2 launch tag_hotspot_nav explore.launch.py cmd_vel_topic:=/dry_run/cmd_vel  # 바퀴 안 굴림
  ros2 launch tag_hotspot_nav explore.launch.py use_safety:=true      # 안전 게이트 삽입
  ros2 launch tag_hotspot_nav explore.launch.py use_sound:=false      # 사운드 끄기

⚠ use_safety:=true 시 체인이 pure_pursuit→/cmd_vel_raw→safety_layer→cmd_vel 로 바뀐다.
   safety_layer 의 정지거리(front/back_stop_dist)는 base_link 기준 PLACEHOLDER 라
   실기에서 한 번 검증/튜닝하기 전엔 기본 off. 검증은 들어올린 상태에서 /scan 앞에
   손 갖다 대며 block 카운터(/safety/state) 증가 확인.
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
    use_cliff_stop = LaunchConfiguration('use_cliff_stop')
    dynamic_clean = LaunchConfiguration('dynamic_clean')

    # map_cleaner 켜면 frontier 는 잔상 제거된 /map_nav 를 본다
    map_topic = PythonExpression(
        ["'/map_nav' if '", use_map_cleaner, "' == 'true' else '/map'"])

    # cliff 핸들링(use_cliff_stop) off 면 map_cleaner 의 /cliff_alert 도 끊어 keep-out 안 생기게
    # (오탐 회피). on 이면 그대로 /cliff_alert 구독 → 계단 keep-out 동작.
    cleaner_cliff_topic = PythonExpression(
        ["'/cliff_alert' if '", use_cliff_stop, "' == 'true' else '/cliff_alert_ignored'"])

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
                              description='최대 선속도 [m/s]'),
        DeclareLaunchArgument('use_safety', default_value='false',
                              description='cmd_vel 안전 게이트(safety_layer) 삽입 — 실기 검증 후 on 권장'),
        DeclareLaunchArgument('use_sound', default_value='true',
                              description='이벤트 사운드(sound_player) 실행'),
        DeclareLaunchArgument('use_map_cleaner', default_value='true',
                              description='동적장애물 잔상 제거(map_cleaner) → frontier 가 /map_nav 사용'),
        DeclareLaunchArgument('use_tag_centering', default_value='true',
                              description='매핑 중 태그 후보에 정렬해 또렷이 포착(tag_centering)'),
        DeclareLaunchArgument('use_cliff_stop', default_value='true',
                              description='젯슨 /cliff_alert → 정지 브리지(cliff_stop)'),
        DeclareLaunchArgument('dynamic_clean', default_value='false',
                              description='map_cleaner 동적 clear+/scan 장애물 마킹(off면 slam맵+keep-out만)'),

        Node(
            package='tag_hotspot_nav',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[{
                # 0.25(5셀, 통로>0.50m) → 0.20(4셀, 통로>0.40m). 본체 0.43m라 0.40~0.43m
                # 통로는 계획에 잡히지만 빡빡함(좁은통로 통과용, 사용자 요청). 5cm격자라 중간값 없음.
                'robot_radius': 0.20,
                'goal_timeout': 100.0,   # 먼 목표도 끝까지 추종(15→100, 사용자 요청). 막힘은 no_progress(30s)가 따로 잡음
                'no_frontier_limit': 5,   # 종료 조건: 진행불가 연속 N회 (기본 30 → 5)
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
                'lookahead': 0.4,
                'goal_tolerance': 0.30,
                # 전방 정지 마진 확대(사용자 요청): base_link 기준 0.60m 에서 정지.
                # 차체 앞단 ~0.26m → 실측 앞단~장애물 간격 ~0.34m 에서 멈춤.
                # slow_down 은 항상 stop 보다 커야 함 → 1.10 부터 감속.
                'stop_dist': 0.60,
                'slow_down_dist': 1.10,
                # 전방 감시 섹터 반각[deg]. 과거 ±30°에서 23° 옆 모서리 빔 오탐 이력 →
                # 넓힐 때 좁은통로/벽앞회전 오정지 관찰. 유리 등 약반사물 포착엔 넓은 게 유리.
                'front_sector_deg': 30.0,
            }],
            remappings=tf_remaps,
        ),

        # 안전 게이트 (옵트인) — /cmd_vel_raw → cmd_vel_topic
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
            }],
        ),

        # 이벤트 사운드 (기본 on) — TF 불요
        Node(
            package='tag_hotspot_nav',
            executable='sound_player',
            name='sound_player',
            output='screen',
            condition=IfCondition(use_sound),
        ),

        # 끼임 감지 (명령 지속 + 휠오돔 무이동 → /stuck) — 사운드 pullup 트리거
        Node(
            package='tag_hotspot_nav',
            executable='stuck_detector',
            name='stuck_detector',
            output='screen',
        ),

        # 낭떠러지/계단 비상정지 (기본 on) — /livox/lidar 점군, TF 불요
        # 계단 감지는 젯슨 front depth(/cliff_alert) 담당. 이 브리지가 받아서 정지시킨다.
        Node(
            package='tag_hotspot_nav',
            executable='cliff_stop',
            name='cliff_stop',
            output='screen',
            condition=IfCondition(use_cliff_stop),
        ),

        # 매핑 중 태그 정렬 (front 후보 → 잠깐 정렬·관측). cmd_vel 은 pure_pursuit 와 동일 토픽
        Node(
            package='tag_hotspot_nav',
            executable='tag_centering',
            name='tag_centering',
            output='screen',
            condition=IfCondition(use_tag_centering),
            parameters=[{'cmd_vel_topic': pp_cmd_topic}],
            remappings=tf_remaps,
        ),

        # 동적장애물 잔상 제거 + /scan 장애물 마킹(dynamic_clean) — /map+/scan → /map_nav, TF 필요.
        # cliff 핸들링 off 면 /cliff_alert 를 죽은 토픽으로 리매핑해 keep-out 비활성(오탐 회피).
        Node(
            package='tag_hotspot_nav',
            executable='map_cleaner',
            name='map_cleaner',
            output='screen',
            condition=IfCondition(use_map_cleaner),
            parameters=[{'dynamic_clean': dynamic_clean}],
            remappings=tf_remaps + [('/cliff_alert', cleaner_cliff_topic)],
        ),
    ])
