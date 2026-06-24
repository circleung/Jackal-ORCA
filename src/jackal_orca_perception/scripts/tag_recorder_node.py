#!/usr/bin/env python3
"""
tag_recorder_node.py
태그 앞 도착 시 apriltag_ros로 ID 읽고 SLAM 좌표와 함께 저장

[디버그 백업 용도] 정식 기록은 mini PC의 tag_mapper(claude.md §7.1 F9)가
camera optical frame → map 변환으로 수행. 이 노드는 검출 순간의 로봇
위치(map→base_link)를 기록하는 단순 백업이며, mini PC SLAM TF가 DDS로
보여야 동작한다.

구독:
  /at_tag_position              (std_msgs/Bool)  — 제어팀: 태그 앞 도착 신호
                                                   auto_record=true면 무시
  /apriltag_{front,back}/detections  (apriltag_msgs/AprilTagDetectionArray)

발행:
  /recorded_tag_positions  (geometry_msgs/PoseArray)    — 저장된 좌표 누적 목록
  /tag_saved_event         (std_msgs/UInt32)            — 새로 저장된 tag_id (자칼 사운드 트리거)

저장:
  ~/ros2_ws/tag_records_MMDD_HHMM.csv  (tag_id, map_x, map_y)

파라미터:
  auto_record (bool, default True): /at_tag_position 신호 없이 apriltag 검출만으로 저장
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Bool, UInt32
from geometry_msgs.msg import PoseArray, Pose
import tf2_ros
import csv
import os
from datetime import datetime


class TagRecorderNode(Node):
    def __init__(self):
        super().__init__('tag_recorder_node')

        self.declare_parameter('auto_record', True)
        self.declare_parameter(
            'detection_topics',
            ['/apriltag_front/detections',
             '/apriltag_back/detections'])
        self.auto_record = self.get_parameter('auto_record').value
        topics = self.get_parameter('detection_topics').value

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.records: list[tuple[int, float, float]] = []  # (tag_id, x, y)
        # auto_record=True면 항상 저장 대기 상태
        self.ready_to_record = self.auto_record

        # CSV 파일 경로
        ts = datetime.now().strftime('%m%d_%H%M')
        self.csv_path = os.path.expanduser(f'~/ros2_ws/tag_records_{ts}.csv')
        with open(self.csv_path, 'w', newline='') as f:
            csv.writer(f).writerow(['tag_id', 'map_x', 'map_y'])

        # ── 구독 ──────────────────────────────────────────────────
        self.create_subscription(Bool, '/at_tag_position',
                                 self._at_tag_cb, 10)

        # apriltag_ros 탐지 (태그 앞 근거리에서 ID 읽기) — 다중 토픽 구독으로
        # 앞/뒤 카메라 detector를 한 노드에서 흡수 (topic_tools relay 불필요)
        try:
            from apriltag_msgs.msg import AprilTagDetectionArray
            for t in topics:
                self.create_subscription(
                    AprilTagDetectionArray, t, self._apriltag_cb, 10)
            self.get_logger().info(f'apriltag_msgs 구독: {topics}')
        except ImportError:
            self.get_logger().warn(
                'apriltag_msgs 없음 — '
                'sudo apt install ros-humble-apriltag-msgs 실행 후 재빌드')

        # ── 발행 ──────────────────────────────────────────────────
        self.pub_positions = self.create_publisher(PoseArray, '/recorded_tag_positions', 10)
        self.pub_event     = self.create_publisher(UInt32,    '/tag_saved_event',        10)

        mode = 'auto_record' if self.auto_record else 'control-triggered'
        self.get_logger().info(
            f'태그 기록 노드 시작 [{mode}] | CSV: {self.csv_path}')

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
            duplicate = False
            for rec_id, rec_x, rec_y in self.records:
                dist = ((rec_x - map_x) ** 2 + (rec_y - map_y) ** 2) ** 0.5
                if rec_id == tag_id and dist < 1.0:
                    duplicate = True
                    break
            if duplicate:
                # auto_record면 다음 검출도 받아야 하므로 ready 유지
                if not self.auto_record:
                    self.ready_to_record = False
                continue

            # 저장
            self.records.append((tag_id, map_x, map_y))
            with open(self.csv_path, 'a', newline='') as f:
                csv.writer(f).writerow([tag_id, round(map_x, 4), round(map_y, 4)])

            self.get_logger().info(
                f'✅ Tag {tag_id} 기록 완료 @ ({map_x:.2f}, {map_y:.2f}) '
                f'| 누적 {len(self.records)}개')

            # 자칼 사운드 트리거
            self.pub_event.publish(UInt32(data=int(tag_id)))

            # 전체 기록 목록 발행 (mine_cluster_node 사용)
            self._publish_positions()

            if not self.auto_record:
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
