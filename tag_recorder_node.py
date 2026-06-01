#!/usr/bin/env python3
"""
tag_recorder_node.py
태그 앞 도착 시 apriltag_ros로 ID 읽고 SLAM 좌표와 함께 저장

구독:
  /at_tag_position         (std_msgs/Bool)              — 제어팀: 태그 앞 도착 신호
  /apriltag/detections     (apriltag_msgs/AprilTagDetectionArray)

발행:
  /recorded_tag_positions  (geometry_msgs/PoseArray)    — 저장된 좌표 목록

저장:
  ~/ros2_ws/tag_records_MMDD_HHMM.csv  (tag_id, map_x, map_y)
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Pose
import tf2_ros
import csv
import os
from datetime import datetime


class TagRecorderNode(Node):
    def __init__(self):
        super().__init__('tag_recorder_node')

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.records: list[tuple[int, float, float]] = []  # (tag_id, x, y)
        self.ready_to_record = False

        # CSV 파일 경로
        ts = datetime.now().strftime('%m%d_%H%M')
        self.csv_path = os.path.expanduser(f'~/ros2_ws/tag_records_{ts}.csv')
        with open(self.csv_path, 'w', newline='') as f:
            csv.writer(f).writerow(['tag_id', 'map_x', 'map_y'])

        # ── 구독 ──────────────────────────────────────────────────
        self.create_subscription(Bool, '/at_tag_position',
                                 self._at_tag_cb, 10)

        # apriltag_ros 탐지 (태그 앞 근거리에서 ID 읽기)
        try:
            from apriltag_msgs.msg import AprilTagDetectionArray
            self.create_subscription(
                AprilTagDetectionArray,
                '/apriltag/detections',
                self._apriltag_cb, 10)
            self.get_logger().info('apriltag_msgs 구독 시작')
        except ImportError:
            self.get_logger().warn(
                'apriltag_msgs 없음 — '
                'sudo apt install ros-humble-apriltag-msgs 실행 후 재빌드')

        # ── 발행 ──────────────────────────────────────────────────
        self.pub_positions = self.create_publisher(PoseArray, '/recorded_tag_positions', 10)

        self.get_logger().info(f'태그 기록 노드 시작 | CSV: {self.csv_path}')

    # ── 콜백 ──────────────────────────────────────────────────────
    def _at_tag_cb(self, msg: Bool):
        """제어팀으로부터 '태그 앞 도착' 신호 수신"""
        if msg.data:
            self.ready_to_record = True
            self.get_logger().info('태그 앞 도착 — ID 대기 중...')

    def _apriltag_cb(self, msg):
        """apriltag_ros 탐지 결과 수신 (근거리, 정면 상태에서 호출됨)"""
        if not self.ready_to_record:
            return
        if not msg.detections:
            return

        for det in msg.detections:
            tag_id = det.id

            # SLAM map 프레임에서 로봇 현재 위치 조회
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_link',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0))
                map_x = tf.transform.translation.x
                map_y = tf.transform.translation.y
            except Exception as e:
                self.get_logger().warn(f'TF 조회 실패: {e}')
                return

            # 중복 기록 방지 (같은 ID를 1m 이내에서 또 본 경우 스킵)
            for rec_id, rec_x, rec_y in self.records:
                dist = ((rec_x - map_x) ** 2 + (rec_y - map_y) ** 2) ** 0.5
                if rec_id == tag_id and dist < 1.0:
                    self.get_logger().info(f'Tag {tag_id} 이미 기록됨 — 스킵')
                    self.ready_to_record = False
                    return

            # 저장
            self.records.append((tag_id, map_x, map_y))
            with open(self.csv_path, 'a', newline='') as f:
                csv.writer(f).writerow([tag_id, round(map_x, 4), round(map_y, 4)])

            self.get_logger().info(
                f'✅ Tag {tag_id} 기록 완료 @ ({map_x:.2f}, {map_y:.2f}) '
                f'| 누적 {len(self.records)}개')

            # 전체 기록 목록 발행 (mine_cluster_node 사용)
            self._publish_positions()

            self.ready_to_record = False  # 다음 신호 대기
            break  # 한 번에 하나만

    def _publish_positions(self):
        pa = PoseArray()
        pa.header.frame_id = 'map'
        pa.header.stamp = self.get_clock().now().to_msg()
        for _, x, y in self.records:
            p = Pose()
            p.position.x = x
            p.position.y = y
            pa.poses.append(p)
        self.pub_positions.publish(pa)


def main(args=None):
    rclpy.init(args=args)
    node = TagRecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
