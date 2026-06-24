"""
explore.launch.py — 자율 탐사 스택.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    linear_speed   = LaunchConfiguration('linear_speed')
    use_safety     = LaunchConfiguration('use_safety')
    use_sound      = LaunchConfiguration('use_sound')
    use_map_cleaner = LaunchConfiguration('use_map_cleaner')
    use_tag_centering = LaunchConfiguration('use_tag_centering')
    dynamic_clean  = LaunchConfiguration('dynamic_clean')

    map_topic = PythonExpression(
        ["'/map_nav' if '", use_map_cleaner, "' == 'true' else '/map'"])

    tf_remaps = [
        ('/tf',        '/j100_0915/tf'),
        ('/tf_static', '/j100_0915/tf_static'),
    ]

    pp_cmd_topic = PythonExpression(
        ["'/cmd_vel_raw' if '", use_safety, "' == 'true' else '", cmd_vel_topic, "'"])

    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic',  default_value='/j100_0915/cmd_vel'),
        DeclareLaunchArgument('linear_speed',   default_value='0.4'),
        DeclareLaunchArgument('use_safety',     default_value='false'),
        DeclareLaunchArgument('use_sound',      default_value='true'),
        DeclareLaunchArgument('use_map_cleaner',default_value='false'),
        DeclareLaunchArgument('use_tag_centering', default_value='false'),  # 태그 보면 멈춰 회전(유턴)하던 동작 OFF — 탐지/저장은 tag_collector가 주행중 수행
        DeclareLaunchArgument('dynamic_clean',  default_value='false'),

        Node(
            package='tag_hotspot_nav',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[{
                'robot_radius': 0.25,         # 0.30→0.25: 0.30(팽창6셀=0.30m)은 문(통로)을 C-space에서 완전 봉인해 로봇과 문너머 frontier가 다른 컴포넌트로 분리→통과 불가(라이브 스윕 확정: 도달영역 5,619→81,102셀). 차체 반폭 0.235m이라 0.25(팽창5셀=0.25m)는 직진 시 1.5cm 여유 유지+cost_map 중앙선호. 실제 충돌방어는 pure_pursuit /scan 정지(0.40m)가 담당
                'goal_timeout': 60.0,
                'no_progress_timeout': 18.0,  # 12→18: 인내심 ↑ → 후진 트리거 빈도 자체를 줄임
                'no_frontier_limit': 15,      # 6→15: 성급한 조기 종료 방지(로컬미니멈 완화)
                'heading_weight': 3.0,         # 1.5→3.0: DFS 강화 — 한 방향(branch) 끝까지 파고듦(갈림길 유턴/반대횡단 억제)
                'size_weight': 0.15,           # 0.4→0.15: 거리(가까움)가 지배 → 가까운 방을 먼저 다 훑고 멀리 감(현재방 우선 커버)
                'revisit_limit': 1,           # 2→1: 방문지점 즉시 재방문 차단(넓은복도 왔다갔다 억제)
                'min_frontier_size': 15,      # 50→15: 문 입구는 탐지하되 노이즈성 자잘틈은 제외
                'min_passage_width': 1.6,     # 폭~1.5m AND 깊이~1.5m 공간만 스킵(사용자 의도). 깊은 골목은 통과(robot_radius로 진입가능)
                'min_passage_depth': 1.6,     # 2.2→1.6: 깊이 1.6m 미만(=약1.5m 이하)만 제외. 깊이 2m 골목 등은 탐사(좁아도 깊으면 들어감)
                'visit_penalty': 3.0,         # 방문영역 인근 frontier 비용 페널티(재방문 차단)
                'visit_radius': 1.0,          # 0.5→1.0: 지나간 복도를 더 넓게 '커버됨' 표시 → 넓은복도 재횡단(왔다갔다) 억제
                'scan_spin_duration': 0.0,     # 목표마다 회전 제거 → 쭉 직진(태그는 주행중 전방카메라가 잡음)
                'cmd_vel_topic': cmd_vel_topic,
                'backup_duration': 0.8,       # 2.0→0.8: 후진 최소화(짧은 넛지만), 실제 탈출은 제자리회전이 담당
                'backup_speed': -0.2,
                'backup_min_rear': 0.40,      # 0.35→0.40: 후방 여유 더 크게 요구 → 후진 더 잘 중단(벽 박기 방지)
                # OFF: 작은 방 입구도 3m 안에 사방이 벽이라 "유리방"과 구분 못 하고
                # 같이 막아버림(실주행에서 매 사이클 11개씩 정상 frontier 차단 확인).
                # 휴리스틱 재설계 전까지 비활성화.
                'enclosure_ratio_threshold': 0.0,
                # 경로 계획 자체가 실패한(path=None) frontier는 1번만 실패해도 바로
                # 영구 차단 — "못 들어간다고 판단해서 다른 곳으로 이동" 직후에도
                # 화면에 점이 계속 남아있던 문제 해결(기존 3번 누적 전까진 TTL만 적용).
                'blacklist_max_strikes': 1,
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
                'lookahead': 0.55,            # 0.9→0.55: 코너 잘라먹기 완화(경로 추종 정확도↑)
                'max_angular': 0.9,           # 0.6→0.9: 곡선 추종 가능(회전 속도↑)
                'goal_tolerance': 0.20,       # 0.30→0.20: 계획점에 더 근접 도달(오blacklist 감소)
                'stop_dist': 0.40,            # 정지 진입(0.50→0.40, range_min 0.20이라 유효)
                'stop_release_dist': 0.55,    # 정지 해제(히스테리시스 → 정지-주행 토글 제거)
                'slow_down_dist': 0.70,       # 0.70m까지 안 감속 → 복도 빠르게 통과
                'heading_slow_angle': 1.8,    # 회전 중 선속도 유지(과감속 방지)
                'front_sector_deg': 25.0,     # 25°로 좁혀 복도 옆벽 오감지 제거
                'rotate_slow_clearance': 0.30, # 복도벽(~0.5m)이 회전 감속 유발 방지
                'rotate_stop_clearance': 0.20, # 극단적 근접 시만 최저속
                'rotate_min_scale': 0.50,      # 최저 회전 50% (기존 25% → 느림 해결)
                'ang_smooth': 0.45,            # 각속도 EMA → 웨이포인트 전환 시 진동 억제
                'rotate_in_place_angle': 1.0,  # 제자리회전 진입 임계
                'rotate_exit_angle': 0.6,      # 해제 임계(히스테리시스) → 경계 떨림/정지-주행 반복 제거
                # DFS 의도: 막힌 벽이 아니면 무조건 가던 방향 유지. 후진은 frontier_explorer의
                # 막힘탈출(backup_duration)만 담당 — pure_pursuit 경로추종 중 후진 선택 금지.
                'allow_reverse': False,
            }],
            remappings=tf_remaps,
        ),

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
                'front_stop_dist': 0.65,   # TV다리 등 얇은 장애물 대비 여유 확보
                'back_stop_dist': 0.40,
                'swept_half_width': 0.235,
            }],
        ),

        Node(
            package='tag_hotspot_nav',
            executable='sound_player',
            name='sound_player',
            output='screen',
            condition=IfCondition(use_sound),
        ),

        Node(
            package='tag_hotspot_nav',
            executable='stuck_detector',
            name='stuck_detector',
            output='screen',
        ),

        Node(
            package='tag_hotspot_nav',
            executable='tag_centering',
            name='tag_centering',
            output='screen',
            condition=IfCondition(use_tag_centering),
            parameters=[{'cmd_vel_topic': pp_cmd_topic}],
            remappings=tf_remaps,
        ),

        Node(
            package='tag_hotspot_nav',
            executable='map_cleaner',
            name='map_cleaner',
            output='screen',
            condition=IfCondition(use_map_cleaner),
            parameters=[{'dynamic_clean': dynamic_clean}],
            remappings=tf_remaps,
        ),

        Node(
            package='tag_hotspot_nav',
            executable='joy_estop',
            name='joy_estop',
            output='screen',
        ),

        # 미션 완료 흐름: 탐사완료(/finish_exploration) → /hotspots(밀집순) 접근 →
        # 도착 후 정지(/final_goal_reached). auto_start=True 로 완전 자율.
        # (이전엔 어떤 launch 에도 없어 탐사 후 클러스터 이동이 동작하지 않았음)
        Node(
            package='tag_hotspot_nav',
            executable='hotspot_navigator',
            name='hotspot_navigator',
            output='screen',
            parameters=[{
                'robot_radius': 0.25,         # frontier_explorer 와 동일 — hotspot 접근도 문 통과 가능해야 함(0.30이면 문 봉인)
                'map_topic': map_topic,
                # OFF: 여러 hotspot 순회 방식이 미션 스펙(태그 평균 1지점)과 안 맞음.
                # finish_mission_listener.py 가 /finish_exploration 을 받아
                # 평균 지점 1곳으로만 이동하도록 대체.
                'auto_start': False,
                'dwell_sec': 2.0,
                'goal_timeout': 40.0,
            }],
            remappings=tf_remaps,
        ),
    ])
