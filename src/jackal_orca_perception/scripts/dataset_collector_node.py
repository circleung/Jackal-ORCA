#!/usr/bin/env python3
"""
dataset_collector_node.py
YOLO 재학습용 데이터셋 수집 — apriltag_ros 검출 결과로 자동 라벨링

mode:=positive  태그를 붙여놓고 주행. apriltag_ros가 해독에 성공한 프레임만
                저장하고, corner 4점 → YOLO bbox 라벨을 자동 생성한다.
                (검출 실패 프레임은 버림 — 독이 든 negative 방지)
mode:=negative  태그를 전부 떼고 주행. 샘플링된 모든 프레임을 빈 라벨로 저장.
                혹시 태그가 검출되면 그 프레임은 건너뛰고 경고만 출력한다.

출력 (YOLO 포맷 풀 — train/val 분할은 학습 직전에 수행):
  <output_dir>/images/<mode>_<session>_NNNNNN.jpg
  <output_dir>/labels/<mode>_<session>_NNNNNN.txt

사용법:
  # apriltag_pipeline.launch.py 가 떠 있는 상태에서:
  ros2 run jackal_orca_perception dataset_collector_node.py --ros-args \
      -p mode:=negative \
      -p image_topic:=/camera_front/color/image_raw \
      -p detections_topic:=/apriltag_front/detections

터미널 명령 (실행 중 입력 + Enter):
  pause   자동 저장 일시정지
  resume  자동 저장 재개
  save    1장만 즉시 저장 (일시정지 중에도 동작; positive는 다음 검출 프레임)
  status  현재 상태/저장 수 출력
start_paused:=true 로 시작하면 일시정지 상태로 시작한다.
"""

import sys
import threading
import time
from collections import OrderedDict
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge


class DatasetCollectorNode(Node):
    def __init__(self):
        super().__init__('dataset_collector_node')

        # ── 파라미터 ──────────────────────────────────────────────
        self.declare_parameter('mode', 'negative')          # positive | negative
        self.declare_parameter('image_topic', '/camera_front/color/image_raw')
        self.declare_parameter('detections_topic', '/apriltag_front/detections')
        # 기본 저장 위치: USB가 마운트돼 있으면 USB, 없으면 홈
        self.declare_parameter('output_dir', self._default_output_dir())
        self.declare_parameter('sample_hz', 2.0)            # 저장 빈도 [Hz]
        self.declare_parameter('jpeg_quality', 95)
        # bbox 여유 — corner는 검정 테두리 기준이라 흰 여백까지 약간 확장
        self.declare_parameter('bbox_margin_ratio', 0.10)
        # true면 일시정지 상태로 시작 — 터미널에 resume/save 입력해서 시작
        self.declare_parameter('start_paused', False)

        self.mode = self.get_parameter('mode').value
        if self.mode not in ('positive', 'negative'):
            raise ValueError(f"mode는 positive|negative 중 하나: {self.mode}")
        self.out_dir = Path(self.get_parameter('output_dir').value)
        self.sample_period = 1.0 / float(self.get_parameter('sample_hz').value)
        self.jpeg_q = int(self.get_parameter('jpeg_quality').value)
        self.margin = float(self.get_parameter('bbox_margin_ratio').value)

        (self.out_dir / 'images').mkdir(parents=True, exist_ok=True)
        (self.out_dir / 'labels').mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.session = time.strftime('%m%d_%H%M%S')
        self.paused = bool(self.get_parameter('start_paused').value)
        self._save_requests = 0      # 'save' 명령으로 쌓이는 즉시 저장 요청 수
        self.saved = 0
        self.skipped_tag_in_negative = 0
        self._last_save_t = 0.0
        # positive 모드: 검출 도착 시 같은 stamp의 이미지를 찾아 저장
        self._img_by_stamp: OrderedDict = OrderedDict()   # (sec, nsec) → cv_img
        self._last_det_stamp = None                       # negative 모드 안전망

        image_topic = self.get_parameter('image_topic').value
        det_topic = self.get_parameter('detections_topic').value
        self.create_subscription(Image, image_topic, self._image_cb, 10)
        self.create_subscription(
            AprilTagDetectionArray, det_topic, self._det_cb, 10)

        self.get_logger().info(
            f'수집 시작 | mode={self.mode} | session={self.session} | '
            f'{self.sample_period:.2f}s 간격 → {self.out_dir}')
        if str(self.out_dir).startswith('/media/'):
            self.get_logger().info('💾 USB에 저장 중')
        else:
            self.get_logger().warn(
                'USB 미탐지 — 홈 디렉토리에 저장. USB를 쓰려면 마운트 후 재시작')

        # ── 터미널 명령 입력 스레드 (pause / resume / save / status) ──
        threading.Thread(target=self._stdin_loop, daemon=True).start()
        state = '⏸ 일시정지 상태로 시작' if self.paused else '▶ 저장 중'
        self.get_logger().info(
            f'{state} — 터미널에 pause / resume / save / status 입력 + Enter')

    # ── 터미널 명령 처리 ─────────────────────────────────────────
    def _stdin_loop(self):
        try:
            for line in sys.stdin:
                cmd = line.strip().lower()
                if cmd == 'pause':
                    self.paused = True
                    self.get_logger().info(f'⏸ 일시정지 (지금까지 {self.saved}장)')
                elif cmd == 'resume':
                    self.paused = False
                    self.get_logger().info('▶ 저장 재개')
                elif cmd in ('save', 's'):
                    self._save_requests += 1
                    hint = ('다음 검출 프레임 1장 저장' if self.mode == 'positive'
                            else '다음 프레임 1장 저장')
                    self.get_logger().info(f'📸 save — {hint}')
                elif cmd == 'status':
                    state = '⏸ 일시정지' if self.paused else '▶ 저장 중'
                    self.get_logger().info(
                        f'{state} | {self.saved}장 저장됨 → {self.out_dir}')
                elif cmd:
                    self.get_logger().warn(
                        f"'{cmd}'? — pause / resume / save / status 중 하나를 입력")
        except Exception:
            pass    # 백그라운드 실행 등 stdin이 없으면 조용히 종료

    # ── 기본 저장 위치: USB 마운트 자동 탐지 ──────────────────────
    @staticmethod
    def _default_output_dir() -> str:
        """/media/jetson/ 아래 쓰기 가능한 USB 마운트가 있으면 그쪽을 기본으로.

        여러 개면 첫 번째(이름순). output_dir 파라미터로 언제든 덮어쓸 수 있다.
        """
        import os
        media = Path('/media/jetson')
        if media.is_dir():
            for mnt in sorted(media.iterdir()):
                if mnt.is_dir() and os.access(mnt, os.W_OK) and os.path.ismount(mnt):
                    return str(mnt / 'apriltag_real')
        return '/home/jetson/datasets/apriltag_real'

    # ── 이미지 콜백 ───────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)

        if self.mode == 'positive':
            # 검출이 이미지보다 늦게 도착하므로 최근 프레임을 버퍼링만 해둔다
            try:
                self._img_by_stamp[key] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().warn(f'이미지 변환 실패: {e}')
                return
            while len(self._img_by_stamp) > 90:             # ~3초 버퍼
                self._img_by_stamp.popitem(last=False)
            return

        # ── negative 모드: 샘플링 주기마다 빈 라벨로 저장 ─────────
        now = time.monotonic()
        force = self._save_requests > 0          # 'save' 명령 — 1장 즉시 저장
        if not force:
            if self.paused:
                return
            if now - self._last_save_t < self.sample_period:
                return
        # 안전망: 직전 1초 내 태그가 검출됐다면 저장하지 않음 (떼다 만 태그 등)
        if self._last_det_stamp is not None and \
                abs((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
                    - self._last_det_stamp) < 1.0:
            self.skipped_tag_in_negative += 1
            self.get_logger().warn(
                f'negative 모드인데 태그 검출됨 — 프레임 건너뜀 '
                f'(누적 {self.skipped_tag_in_negative}장). 태그를 떼었는지 확인!',
                throttle_duration_sec=2.0)
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'이미지 변환 실패: {e}')
            return
        self._last_save_t = now
        self._save(cv_img, [])
        if force:
            self._save_requests -= 1
            self.get_logger().info(f'📸 저장 완료 ({self.saved}장)')

    # ── 검출 콜백 ─────────────────────────────────────────────────
    def _det_cb(self, msg: AprilTagDetectionArray):
        if len(msg.detections) == 0:
            return
        stamp_f = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._last_det_stamp = stamp_f                      # negative 안전망용

        if self.mode != 'positive':
            return
        now = time.monotonic()
        force = self._save_requests > 0          # 'save' 명령 — 1장 즉시 저장
        if not force:
            if self.paused:
                return
            if now - self._last_save_t < self.sample_period:
                return
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        cv_img = self._img_by_stamp.get(key)
        if cv_img is None:
            return                                          # 버퍼에서 밀려남 — 드묾
        h, w = cv_img.shape[:2]
        labels = []
        for det in msg.detections:
            xs = [c.x for c in det.corners]
            ys = [c.y for c in det.corners]
            x1, x2 = min(xs), max(xs)
            y1, y2 = min(ys), max(ys)
            # 흰 여백 포함하도록 약간 확장
            mx = (x2 - x1) * self.margin
            my = (y2 - y1) * self.margin
            x1, x2 = max(0.0, x1 - mx), min(float(w), x2 + mx)
            y1, y2 = max(0.0, y1 - my), min(float(h), y2 + my)
            cx, cy = (x1 + x2) / 2 / w, (y1 + y2) / 2 / h
            bw, bh = (x2 - x1) / w, (y2 - y1) / h
            if bw <= 0 or bh <= 0:
                continue
            labels.append(f'0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}')
        if not labels:
            return
        self._last_save_t = now
        self._save(cv_img, labels)
        if force:
            self._save_requests -= 1
            self.get_logger().info(f'📸 저장 완료 ({self.saved}장)')

    # ── 저장 ──────────────────────────────────────────────────────
    def _save(self, cv_img, labels: list):
        stem = f'{self.mode[:3]}_{self.session}_{self.saved:06d}'
        cv2.imwrite(str(self.out_dir / 'images' / f'{stem}.jpg'), cv_img,
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_q])
        (self.out_dir / 'labels' / f'{stem}.txt').write_text(
            '\n'.join(labels) + ('\n' if labels else ''))
        self.saved += 1
        if self.saved % 25 == 0:
            self.get_logger().info(f'[{self.mode}] {self.saved}장 저장됨')


def main(args=None):
    rclpy.init(args=args)
    node = DatasetCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.get_logger().info(
        f'수집 종료 — {node.mode} {node.saved}장 → {node.out_dir}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
