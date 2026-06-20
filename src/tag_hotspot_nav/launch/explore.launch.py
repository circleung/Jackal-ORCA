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
                'no_progress_timeout': 8.0,   # 8s 막힘 → 즉시 재계획 (기존 30s)
                'no_frontier_limit': 3,
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
                'lookahead': 1.0,             # 1.0m → 진동 없는 부드러운 추종
                'max_angular': 0.6,           # 회전 부드럽게
                'goal_tolerance': 0.30,
                'stop_dist': 0.55,            # 0.55m → TV 다리 등 얇은 장애물
                'slow_down_dist': 1.10,
                'front_sector_deg': 35.0,     # ±35° 전방 감시
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
                'front_stop_dist': 0.50,
                'back_stop_dist': 0.50,
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
