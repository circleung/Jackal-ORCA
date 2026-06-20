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
        DeclareLaunchArgument('use_tag_centering', default_value='true'),
        DeclareLaunchArgument('dynamic_clean',  default_value='false'),

        Node(
            package='tag_hotspot_nav',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[{
                'robot_radius': 0.22,
                'goal_timeout': 60.0,
                'no_progress_timeout': 12.0,  # 막힘 인내심 ↑ (경로 유지). 첫 무진전은 같은 목표 재시도
                'no_frontier_limit': 6,       # 종료/스윕 판정 완화 (blacklist 잦은 리셋 방지)
                'heading_weight': 4.0,         # DFS: 현재 방향 4배 우선
                'scan_spin_duration': 0.0,     # 탐사 중 스핀 off
                'cmd_vel_topic': cmd_vel_topic,
                'backup_duration': 2.0,
                'backup_speed': -0.2,
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
                'lookahead': 0.9,             # 1.2→0.9: 코너 타겟 점프 완화(진동·과회전 억제)
                'max_angular': 0.6,
                'goal_tolerance': 0.30,
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
    ])
