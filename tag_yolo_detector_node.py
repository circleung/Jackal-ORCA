#!/usr/bin/env python3
"""
tag_yolo_detector_node.py
RealSense 앞/뒤 카메라로 YOLO AprilTag 탐지 → confidence 발행

구독:
  /camera_front/color/image_raw  (sensor_msgs/Image)
  /camera_rear/color/image_raw   (sensor_msgs/Image)

발행:
  /tag_confidence  (std_msgs/Float32)  — YOLO 최대 confidence (0~1)
  /tag_detected    (std_msgs/Bool)     — threshold 초과 이벤트
  /tag_bbox        (std_msgs/String)   — JSON: {camera, conf, cx, cy, w, h} (디버그용)
"""

import os
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool, String
from cv_bridge import CvBridge


class TagYoloDetectorNode(Node):
    def __init__(self):
        super().__init__('tag_yolo_detector_node')

        # ── 파라미터 ──────────────────────────────────────────────
        # 기본값: 패키지 share 디렉토리의 models 폴더
        try:
            from ament_index_python.packages import get_package_share_directory
            _default_model = os.path.join(
                get_package_share_directory('jackal_mine_detection'),
                'models', 'apriltag_yolo.pt')
        except Exception:
            _default_model = os.path.expanduser(
                '~/ros2_ws/src/jackal_mine_detection/models/apriltag_yolo.pt')
        self.declare_parameter('model_path', _default_model)
        self.declare_parameter('confidence_threshold', 0.55)
        self.declare_parameter('front_topic', '/camera_front/color/image_raw')
        self.declare_parameter('rear_topic',  '/camera_rear/color/image_raw')
        self.declare_parameter('inference_every_n_frames', 3)  # 연산 부하 조절

        model_path = self.get_parameter('model_path').value
        self.threshold = self.get_parameter('confidence_threshold').value
        front_topic = self.get_parameter('front_topic').value
        rear_topic  = self.get_parameter('rear_topic').value
        self.infer_interval = self.get_parameter('inference_every_n_frames').value

        # ── YOLO 모델 로드 ────────────────────────────────────────
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO 모델 로드 완료: {model_path}')
        except Exception as e:
            self.get_logger().error(f'YOLO 로드 실패: {e}')
            raise

        self.bridge = CvBridge()
        self._frame_cnt = {'front': 0, 'rear': 0}

        # ── 구독 ──────────────────────────────────────────────────
        self.create_subscription(
            Image, front_topic,
            lambda msg: self._image_cb(msg, 'front'), 10)
        self.create_subscription(
            Image, rear_topic,
            lambda msg: self._image_cb(msg, 'rear'), 10)

        # ── 발행 ──────────────────────────────────────────────────
        self.pub_conf     = self.create_publisher(Float32, '/tag_confidence', 10)
        self.pub_detected = self.create_publisher(Bool,    '/tag_detected',   10)
        self.pub_bbox     = self.create_publisher(String,  '/tag_bbox',       10)

        self.get_logger().info(
            f'태그 탐지 노드 시작 | threshold={self.threshold} | '
            f'front={front_topic} | rear={rear_topic}')

    def _image_cb(self, msg: Image, cam_id: str):
        # N프레임마다 추론 (실시간 부하 조절)
        self._frame_cnt[cam_id] += 1
        if self._frame_cnt[cam_id] % self.infer_interval != 0:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'이미지 변환 실패: {e}')
            return

        results = self.model(cv_img, verbose=False)

        max_conf = 0.0
        best_box = None

        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf > max_conf:
                    max_conf = conf
                    xyxy = box.xyxy[0].tolist()
                    best_box = xyxy

        # confidence 발행
        self.pub_conf.publish(Float32(data=max_conf))

        # threshold 초과 → detected 이벤트
        if max_conf >= self.threshold:
            self.pub_detected.publish(Bool(data=True))
            self.get_logger().info(
                f'[{cam_id}] 태그 탐지! conf={max_conf:.2f}')

        # bbox 정보 발행 (디버그 / 제어팀 사용)
        if best_box is not None:
            h, w = cv_img.shape[:2]
            x1, y1, x2, y2 = best_box
            bbox_info = {
                'camera': cam_id,
                'conf': round(max_conf, 3),
                'cx': round((x1 + x2) / 2 / w, 4),
                'cy': round((y1 + y2) / 2 / h, 4),
                'bw': round((x2 - x1) / w, 4),
                'bh': round((y2 - y1) / h, 4),
            }
            self.pub_bbox.publish(String(data=json.dumps(bbox_info)))


def main(args=None):
    rclpy.init(args=args)
    node = TagYoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
