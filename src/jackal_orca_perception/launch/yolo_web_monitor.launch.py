"""YOLO bbox web monitor (Jetson side).

apriltag_pipeline(enable_yolo:=true) + web_video_server 를 한 번에 띄워
앞/뒤 카메라의 YOLO 바운딩박스 오버레이 영상을 브라우저로 보기 위한 런치.

의도적으로 최소 스트림만 켠다:
  - depth / infra / imu(gyro·accel) : OFF (apriltag_pipeline._rs 에서 비활성)
  - color : ON (apriltag_node·YOLO 의 입력이라 필수)
  - 화면에 보는 것은 /yolo/debug_image_{front,back} (color 위에 bbox 오버레이)

실행:
    ros2 launch jackal_orca_perception yolo_web_monitor.launch.py

브라우저:
    http://<jetson-ip>:8080/stream?topic=/yolo/debug_image_front
    http://<jetson-ip>:8080/stream?topic=/yolo/debug_image_back
    (전체 목록: http://<jetson-ip>:8080 )

인자:
    port (default 8080)         web_video_server 포트
    enable_back (default true)  뒤 카메라 스택
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('jackal_orca_perception')
    pipeline_launch = os.path.join(pkg_share, 'launch', 'apriltag_pipeline.launch.py')

    port        = LaunchConfiguration('port')
    enable_back = LaunchConfiguration('enable_back')

    # 카메라 + apriltag + YOLO. enable_yolo는 항상 true로 고정.
    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pipeline_launch),
        launch_arguments={
            'enable_yolo': 'true',
            'enable_back': enable_back,
        }.items(),
    )

    # bbox 오버레이 영상을 MJPEG로 서빙. 모든 image 토픽을 자동 노출하지만
    # 우리가 여는 것은 /yolo/debug_image_{front,back} 뿐.
    #
    # 경량화: default_stream_type=mjpeg 로 고정 (png는 무손실이라 더 무겁다).
    #   JPEG 압축 품질(quality)·해상도(width/height)는 web_video_server에서
    #   노드 파라미터가 아니라 *URL 쿼리 파라미터*다. 따라서 저품질 기본값은
    #   런치가 아니라 접속 URL에 박는다 (README 표준 URL 참조):
    #     .../stream?topic=...&type=mjpeg&quality=50&width=640
    web = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{
            'port': port,
            'address': '0.0.0.0',
            'default_stream_type': 'mjpeg',   # ?type= 생략 시 mjpeg 사용
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080',
                              description='web_video_server HTTP port'),
        DeclareLaunchArgument('enable_back', default_value='true',
                              description='뒤 카메라/apriltag 스택 동시 기동'),
        pipeline,
        web,
    ])
