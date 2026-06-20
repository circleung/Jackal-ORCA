import os
from glob import glob
from setuptools import setup

package_name = 'tag_hotspot_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.json') + glob('config/*.rviz')),
        # 터미널 명령: install/<pkg>/bin 은 source 시 PATH 에 추가됨
        ('bin', ['scripts/go', 'scripts/pause', 'scripts/save', 'scripts/reset']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jackal',
    maintainer_email='eximy590@jbnu.ac.kr',
    description='2D SLAM 기반 태그 밀집 클러스터링 자율탐사',
    license='MIT',
    entry_points={
        'console_scripts': [
            'frontier_explorer = tag_hotspot_nav.frontier_explorer:main',
            'pure_pursuit = tag_hotspot_nav.pure_pursuit:main',
            # 탐사 제어 명령 (워크스페이스 source 후 터미널에서 바로 사용)
            'go = tag_hotspot_nav.explore_cmd:go',
            'pause = tag_hotspot_nav.explore_cmd:pause',
            'resume = tag_hotspot_nav.explore_cmd:resume',
            'save = tag_hotspot_nav.explore_cmd:save',
            'reset = tag_hotspot_nav.explore_cmd:reset',
            # 2단계: 젯슨 apriltag → solvePnP → map 누적
            'tag_collector = tag_hotspot_nav.tag_collector:main',
            # 안전·사운드 (jackal_mine_detection 에서 2D 스택에 맞게 이관)
            'safety_layer = tag_hotspot_nav.safety_layer:main',
            'sound_player = tag_hotspot_nav.sound_player:main',
            # 동적장애물 잔상 제거
            'map_cleaner = tag_hotspot_nav.map_cleaner:main',
            # 매핑 중 태그 정렬(능동 포착)
            'tag_centering = tag_hotspot_nav.tag_centering:main',
            # 끼임 감지 (명령 지속 + 휠오돔 무이동 → /stuck)
            'stuck_detector = tag_hotspot_nav.stuck_detector:main',
            # 시각화: base_link 에 자칼 부피 박스 Marker (Foxglove URDF 메시 에러 우회)
            'footprint_marker = tag_hotspot_nav.footprint_marker:main',
            # 3단계: 누적 태그 DBSCAN 군집화 → /hotspots
            'clustering = tag_hotspot_nav.clustering:main',
            # 4단계: hotspot 순차 접근 FSM (A*+pure_pursuit 재사용)
            'hotspot_navigator = tag_hotspot_nav.hotspot_navigator:main',
            # PS4 X버튼 비상정지 — platform/safety_stop + /pause 토글
            'joy_estop = tag_hotspot_nav.joy_estop:main',
        ],
    },
)
