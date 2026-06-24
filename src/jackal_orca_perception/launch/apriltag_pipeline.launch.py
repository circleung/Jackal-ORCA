"""AprilTag pipeline launch (Jetson side).

Brings up two RealSense D435i color streams and two apriltag_ros detectors,
plus the optional YOLO tag detector and the tag_recorder_node (debug backup —
authoritative recording is the mini PC's tag_mapper, see claude.md §7.1 F9).

Interface contract (jackal_project_shared/claude.md §5.1):
    /camera_front/color/image_raw      sensor_msgs/Image
    /camera_front/color/camera_info    sensor_msgs/CameraInfo
    /camera_back/color/image_raw       sensor_msgs/Image
    /camera_back/color/camera_info     sensor_msgs/CameraInfo
    /apriltag_front/detections         apriltag_msgs/AprilTagDetectionArray
    /apriltag_back/detections          apriltag_msgs/AprilTagDetectionArray

Frame names follow the contract TF tree (§4):
    camera_front_color_optical_frame / camera_back_color_optical_frame
    (derived from camera_name; realsense2_camera publishes
     <camera_name>_link → <camera_name>_color_optical_frame internally)

NOTE: base_link → camera_{front,back}_link static TFs are owned by the
Jackal mini PC URDF — the Jetson publishes NO TF.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Default values — override at the CLI with `ros2 launch ... front_serial:=...`
# Wrap in literal single-quotes; rs_launch.py treats serial_no as string only if
# the value text contains the quotes (otherwise the param is coerced to int).
DEFAULT_FRONT_SERIAL = "'344522070059'"
DEFAULT_BACK_SERIAL  = "'344522070202'"


def generate_launch_description():
    pkg_share = get_package_share_directory('jackal_orca_perception')
    apriltag_cfg = os.path.join(pkg_share, 'config', 'apriltag.yaml')

    rs_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch')

    front_serial = LaunchConfiguration('front_serial')
    back_serial  = LaunchConfiguration('back_serial')
    enable_back  = LaunchConfiguration('enable_back')
    enable_yolo  = LaunchConfiguration('enable_yolo')
    auto_record  = LaunchConfiguration('auto_record')

    # ── RealSense × 2 ───────────────────────────────────────────────
    # rs_launch publishes under /<camera_namespace>/<camera_name>/...
    # Contract topics are /camera_{front,back}/color/..., so we use an
    # empty namespace and camera_name=camera_{front,back}. This also gives
    # unique TF frames: camera_front_color_optical_frame vs
    # camera_back_color_optical_frame (was a collision when both sides
    # used camera_name='camera').
    def _rs(side: str, serial):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rs_launch_dir, 'rs_launch.py')),
            launch_arguments={
                'serial_no': serial,
                'camera_namespace': '/',
                'camera_name': f'camera_{side}',
                'enable_color':  'true',
                'enable_depth':  'false',   # color-only — AprilTag/YOLO 입력만 사용
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_gyro':   'false',
                'enable_accel':  'false',
            }.items(),
        )

    realsense_front = _rs('front', front_serial)

    # ── apriltag_ros (front) ────────────────────────────────────────
    # apriltag_node subscribes to `image_rect` and `camera_info`, publishes
    # to `detections`. We remap each to per-side contract topics.
    apriltag_front = Node(
        package='apriltag_ros', executable='apriltag_node',
        name='apriltag_front',
        parameters=[apriltag_cfg],
        remappings=[
            ('image_rect',  '/camera_front/color/image_raw'),
            ('camera_info', '/camera_front/color/camera_info'),
            ('detections',  '/apriltag_front/detections'),
        ],
    )

    # ── tag_recorder_node (debug backup) ───────────────────────────
    # Authoritative map-frame recording happens in the mini PC tag_mapper.
    recorder = Node(
        package='jackal_orca_perception',
        executable='tag_recorder_node.py',
        name='tag_recorder_node',
        parameters=[{
            'auto_record': ParameterValue(auto_record, value_type=bool),
        }],
        output='screen',
    )

    # ── tag_yolo_detector_node (optional) ──────────────────────────
    yolo = Node(
        package='jackal_orca_perception',
        executable='tag_yolo_detector_node.py',
        name='tag_yolo_detector_node',
        parameters=[{
            'front_topic': '/camera_front/color/image_raw',
            'back_topic':  '/camera_back/color/image_raw',
        }],
        condition=IfCondition(enable_yolo),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('front_serial', default_value=DEFAULT_FRONT_SERIAL,
                              description='RealSense D435i serial for front camera'),
        DeclareLaunchArgument('back_serial',  default_value=DEFAULT_BACK_SERIAL,
                              description='RealSense D435i serial for back camera'),
        DeclareLaunchArgument('enable_back',  default_value='true',
                              description='Launch the back camera/apriltag stack'),
        DeclareLaunchArgument('enable_yolo',  default_value='false',
                              description='Launch tag_yolo_detector_node (needs ultralytics)'),
        DeclareLaunchArgument('auto_record',  default_value='true',
                              description='Save tags without waiting for /at_tag_position'),

        realsense_front,
        apriltag_front,

        # Back stack only when enabled
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rs_launch_dir, 'rs_launch.py')),
            launch_arguments={
                'serial_no': back_serial,
                'camera_namespace': '/',
                'camera_name': 'camera_back',
                'enable_color': 'true', 'enable_depth': 'false',
                'enable_infra1': 'false', 'enable_infra2': 'false',
                'enable_gyro': 'false', 'enable_accel': 'false',
            }.items(),
            condition=IfCondition(enable_back),
        ),
        Node(
            package='apriltag_ros', executable='apriltag_node',
            name='apriltag_back',
            parameters=[apriltag_cfg],
            remappings=[
                ('image_rect',  '/camera_back/color/image_raw'),
                ('camera_info', '/camera_back/color/camera_info'),
                ('detections',  '/apriltag_back/detections'),
            ],
            condition=IfCondition(enable_back),
        ),
        recorder,
        yolo,
    ])
