#!/usr/bin/env python3
"""
tag_yolo_detector_node.py
RealSense 앞/뒤 카메라로 YOLO AprilTag 탐지 → 방향(bearing) + confidence 발행

mini PC mission_node가 이 노드의 /yolo/tag_candidate 를 받아
TAG_FOUND → APPROACHING_TAG 시각 서보잉을 수행한다 (claude.md §0.9).
Jetson은 TF를 전혀 사용하지 않는다 — bearing은 camera_info intrinsic과
카메라 장착 yaw 파라미터만으로 계산.

구독:
  /camera_front/color/image_raw    (sensor_msgs/Image)
  /camera_front/color/camera_info  (sensor_msgs/CameraInfo)
  /camera_back/color/image_raw     (sensor_msgs/Image)
  /camera_back/color/camera_info   (sensor_msgs/CameraInfo)

발행:
  /yolo/tag_candidate  (custom_msgs/TagCandidate) — bearing_rad(base_link 기준),
                        confidence, source_camera. 서보잉 피드백 스트림.
  /tag_confidence      (std_msgs/Float32)  — YOLO 최대 confidence (디버그)
  /tag_detected        (std_msgs/Bool)     — threshold 초과 이벤트
  /yolo/debug_image_front, /yolo/debug_image_back (sensor_msgs/Image)
                       — bbox 오버레이 영상 (web_video_server 모니터링용,
                         구독자 있을 때만 그리기/발행)

bearing 계산:
  bearing = mount_yaw + atan2(cx_intrinsic - u_px, fx)
  u가 영상 오른쪽일수록 로봇 기준 오른쪽(-yaw). back 카메라는 mount_yaw=pi.
  camera_info 수신 전에는 candidate 발행 안 함 (intrinsic 없이 추정 금지).
"""

import math
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge

from custom_msgs.msg import TagCandidate


class TagYoloDetectorNode(Node):
    def __init__(self):
        super().__init__('tag_yolo_detector_node')

        # ── 파라미터 ──────────────────────────────────────────────
        import os
        try:
            from ament_index_python.packages import get_package_share_directory
            _models_dir = os.path.join(
                get_package_share_directory('jackal_orca_perception'), 'models')
        except Exception:
            _models_dir = os.path.expanduser(
                '~/ros2_ws/src/jackal_orca_perception/models')
        _default_model = self._resolve_model(_models_dir)
        self.declare_parameter('model_path', _default_model)
        self.declare_parameter('confidence_threshold', 0.55)   # /tag_detected 이벤트
        self.declare_parameter('candidate_threshold', 0.30)    # /yolo/tag_candidate 발행
        self.declare_parameter('front_topic', '/camera_front/color/image_raw')
        self.declare_parameter('back_topic',  '/camera_back/color/image_raw')
        self.declare_parameter('front_info_topic', '/camera_front/color/camera_info')
        self.declare_parameter('back_info_topic',  '/camera_back/color/camera_info')
        self.declare_parameter('front_mount_yaw', 0.0)         # base_link 기준 [rad]
        self.declare_parameter('back_mount_yaw',  math.pi)
        self.declare_parameter('inference_every_n_frames', 1)  # 연산 부하 조절

        model_path = self.get_parameter('model_path').value
        self.threshold      = self.get_parameter('confidence_threshold').value
        self.cand_threshold = self.get_parameter('candidate_threshold').value
        front_topic = self.get_parameter('front_topic').value
        back_topic  = self.get_parameter('back_topic').value
        front_info  = self.get_parameter('front_info_topic').value
        back_info   = self.get_parameter('back_info_topic').value
        self.mount_yaw = {
            'front': float(self.get_parameter('front_mount_yaw').value),
            'back':  float(self.get_parameter('back_mount_yaw').value),
        }
        self.infer_interval = self.get_parameter('inference_every_n_frames').value

        # ── YOLO 모델 로드 ────────────────────────────────────────
        try:
            from ultralytics import YOLO
            # .engine은 메타데이터가 없어 task 자동 추론이 안 될 수 있음
            self.model = YOLO(model_path, task='detect')
            backend = 'TensorRT' if model_path.endswith('.engine') else 'PyTorch'
            self.get_logger().info(f'YOLO 모델 로드 완료 ({backend}): {model_path}')
        except Exception as e:
            self.get_logger().error(f'YOLO 로드 실패: {e}')
            raise

        self.bridge = CvBridge()
        self._frame_cnt = {'front': 0, 'back': 0}
        # camera_info에서 받은 intrinsic: {cam_id: (fx, cx)}
        self._intrinsics: dict[str, tuple[float, float]] = {}

        # ── 구독 ──────────────────────────────────────────────────
        self.create_subscription(
            Image, front_topic,
            lambda msg: self._image_cb(msg, 'front'), 10)
        self.create_subscription(
            Image, back_topic,
            lambda msg: self._image_cb(msg, 'back'), 10)
        self.create_subscription(
            CameraInfo, front_info,
            lambda msg: self._info_cb(msg, 'front'), 10)
        self.create_subscription(
            CameraInfo, back_info,
            lambda msg: self._info_cb(msg, 'back'), 10)

        # ── 발행 ──────────────────────────────────────────────────
        self.pub_candidate = self.create_publisher(TagCandidate, '/yolo/tag_candidate', 10)
        self.pub_conf      = self.create_publisher(Float32, '/tag_confidence', 10)
        self.pub_detected  = self.create_publisher(Bool,    '/tag_detected',   10)
        # 디버그 오버레이 영상 (web_video_server 모니터링용)
        self.pub_debug = {
            'front': self.create_publisher(Image, '/yolo/debug_image_front', 10),
            'back':  self.create_publisher(Image, '/yolo/debug_image_back',  10),
        }

        self.get_logger().info(
            f'태그 탐지 노드 시작 | detect_th={self.threshold} '
            f'cand_th={self.cand_threshold} | front={front_topic} | back={back_topic}')

    # ── 모델 파일 선택: TensorRT 엔진 우선 ────────────────────────
    @staticmethod
    def _resolve_model(models_dir: str) -> str:
        """.engine(TensorRT)이 .pt보다 새것이면 우선 사용.

        .pt만 갱신하고 export_trt.py를 안 돌린 경우 — 옛 엔진을 로드하지
        않도록 mtime을 비교해 .pt로 폴백한다.
        """
        import os
        pt = os.path.join(models_dir, 'apriltag_yolo.pt')
        engine = os.path.join(models_dir, 'apriltag_yolo.engine')
        if os.path.exists(engine):
            if not os.path.exists(pt) or os.path.getmtime(engine) >= os.path.getmtime(pt):
                return engine
        return pt

    # ── camera_info: intrinsic 1회 저장 ───────────────────────────
    def _info_cb(self, msg: CameraInfo, cam_id: str):
        if cam_id in self._intrinsics:
            return
        fx = msg.k[0]
        cx = msg.k[2]
        if fx <= 0.0:
            self.get_logger().warn(f'[{cam_id}] camera_info fx<=0, 무시')
            return
        self._intrinsics[cam_id] = (fx, cx)
        self.get_logger().info(f'[{cam_id}] intrinsic 수신: fx={fx:.1f}, cx={cx:.1f}')

    # ── 영상 콜백 ─────────────────────────────────────────────────
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
        all_boxes = []                           # (xyxy, conf) — 디버그 오버레이용

        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue
            for box in r.boxes:
                conf = float(box.conf[0])
                xyxy = box.xyxy[0].tolist()
                all_boxes.append((xyxy, conf))
                if conf > max_conf:
                    max_conf = conf
                    best_box = xyxy

        # 디버그 오버레이 영상 (구독자 있을 때만 — 평시 부하 0)
        if self.pub_debug[cam_id].get_subscription_count() > 0:
            self._publish_debug(cv_img, all_boxes, cam_id, msg.header)

        # confidence 발행 (디버그)
        self.pub_conf.publish(Float32(data=max_conf))

        # threshold 초과 → detected 이벤트
        if max_conf >= self.threshold:
            self.pub_detected.publish(Bool(data=True))
            self.get_logger().info(
                f'[{cam_id}] 태그 탐지! conf={max_conf:.2f}',
                throttle_duration_sec=1.0)

        # ── TagCandidate 발행 (서보잉 피드백) ─────────────────────
        if best_box is None or max_conf < self.cand_threshold:
            return
        if cam_id not in self._intrinsics:
            self.get_logger().warn(
                f'[{cam_id}] camera_info 미수신 — bearing 계산 불가, candidate 스킵',
                throttle_duration_sec=5.0)
            return

        fx, cx = self._intrinsics[cam_id]
        x1, _, x2, _ = best_box
        u = (x1 + x2) / 2.0                      # bbox 중심 픽셀 (가로)
        bearing = self.mount_yaw[cam_id] + math.atan2(cx - u, fx)
        # [-pi, pi) 정규화
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))

        cand = TagCandidate()
        cand.header.stamp = msg.header.stamp     # 원본 이미지 stamp 유지
        cand.header.frame_id = 'base_link'
        cand.source_camera = cam_id
        cand.bearing_rad = bearing
        cand.range_m = -1.0                      # RGB-only — depth 도입 시 채움
        cand.confidence = max_conf
        self.pub_candidate.publish(cand)

    # ── 디버그 오버레이 영상 발행 ─────────────────────────────────
    def _publish_debug(self, cv_img, all_boxes, cam_id: str, header):
        vis = cv_img.copy()
        for xyxy, conf in all_boxes:
            x1, y1, x2, y2 = map(int, xyxy)
            # 탐지 확정(녹색) / 후보(노란색) / 미달(회색)
            if conf >= self.threshold:
                color = (0, 255, 0)
            elif conf >= self.cand_threshold:
                color = (0, 220, 255)
            else:
                color = (160, 160, 160)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            cv2.putText(vis, f'tag {conf:.2f}', (x1, max(y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        cv2.putText(vis, f'{cam_id} | tags: {len(all_boxes)}', (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        out = self.bridge.cv2_to_imgmsg(vis, 'bgr8')
        out.header = header
        self.pub_debug[cam_id].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TagYoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
