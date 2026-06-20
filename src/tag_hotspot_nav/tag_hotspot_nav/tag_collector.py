"""
tag_collector.py — 매핑 중 벽 태그를 map 프레임에 누적하는 노드 (2단계).

젯슨이 발행하는 apriltag 검출(/apriltag_{front,back}/detections, apriltag_msgs)에는
pose 가 없고 corners[4]+homography 만 있다. 따라서 여기서 직접 pose 를 푼다:

알고리즘:
  1. apriltag corners(픽셀) + camera_info(K,D) + 태그 실측 크기 → cv2.solvePnP
     → 카메라 optical frame 기준 태그 중심 3D 위치 (tvec)
  2. tf2 로 optical → map 변환  (★ /tf→/j100_0915/tf 리매핑 필수, launch 에서 적용)
     - 기본은 검출 stamp 시점 TF 로 변환(주행 중 드리프트 방지). 시간 비동기로
       실패하면 최신 TF(stamp=0)로 1회 폴백. use_latest_tf=True 면 항상 최신 TF.
  3. tag_id 별 EMA 누적 → 러닝 추정치 /tags_in_map(TagPoseArray) 발행
  4. 원시 관측 전부를 tag_observations.json 으로 저장 → 3단계 DBSCAN/median 입력

주의:
  - 우리 네비는 2D 라서 클러스터/주행엔 (x,y)만 쓴다. z 는 기록만.
  - solvePnP 의 tag_size 는 "검은 외곽 사각형 변 길이"(흰 여백 제외). 파라미터로 보정.
  - 코너 순서가 라이브러리마다 달라도, 대칭 object point + 중심 위치만 쓰므로
    위치 추정은 순서에 강건하다(자세는 틀어질 수 있으나 미사용).

선행: 젯슨 인식 파이프라인 + slam_2d(맵·TF) + 카메라 base_link→optical static TF.
"""

import json
import math
import os

import numpy as np

try:
    import cv2
except ImportError:  # 빌드 환경엔 없을 수 있어 import 실패해도 노드 메타는 살린다
    cv2 = None

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Int32
from builtin_interfaces.msg import Time as TimeMsg

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  (PointStamped 의 do_transform 등록용)

from custom_msgs.msg import TagPose, TagPoseArray


class TagCollectorNode(Node):

    def __init__(self):
        super().__init__('tag_collector')

        # ── 파라미터 ─────────────────────────────────────────────
        self.declare_parameter('tag_size', 0.14)          # [m] 검은 사각형 변 길이
        self.declare_parameter('map_frame', 'map')
        # False(기본): 검출 stamp 시점 TF 로 변환(주행 드리프트 방지, 실패 시 최신 TF 폴백)
        # True: 항상 최신 TF(stamp=0) — 시간 동기가 깨졌을 때 강제 회피용
        self.declare_parameter('use_latest_tf', False)
        self.declare_parameter('tf_timeout', 0.1)         # [s] stamp TF 버퍼 대기 한도
        self.declare_parameter('raw_cap', 1000)           # 태그별 원시관측 보관 상한 (median용)
        self.declare_parameter('max_range', 6.0)          # [m] 이 이상 멀면 신뢰 안 함(원거리 노이즈)
        self.declare_parameter('min_decision_margin', 30.0)  # apriltag 품질 하한
        self.declare_parameter('publish_rate', 2.0)       # [Hz] /tags_in_map
        self.declare_parameter('save_period', 5.0)        # [s] json 저장 주기
        self.declare_parameter('output_path',
                               os.path.expanduser('~/colcon_ws/tag_observations.json'))

        self.tag_size = float(self.get_parameter('tag_size').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.use_latest_tf = bool(self.get_parameter('use_latest_tf').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.raw_cap = int(self.get_parameter('raw_cap').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_margin = float(self.get_parameter('min_decision_margin').value)
        self.output_path = self.get_parameter('output_path').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        save_period = float(self.get_parameter('save_period').value)

        if cv2 is None:
            self.get_logger().error('cv2(OpenCV) import 실패 — solvePnP 불가. python3-opencv 설치 필요')

        # solvePnP object points: 태그 중심 원점, 평면 z=0, 대칭 배치
        s = self.tag_size / 2.0
        self.object_points = np.array([
            [-s,  s, 0.0],   # 코너 순서는 자세에만 영향, 중심 위치엔 무관
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float64)

        # ── TF ──────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── 카메라 intrinsics (source 별) ─────────────────────────
        self.cam_K = {}   # 'front'/'back' → 3x3
        self.cam_D = {}   # 'front'/'back' → distortion

        # ── 누적 상태: tag_id → dict ─────────────────────────────
        #   {'count':n, 'last_seen':TimeMsg, 'source':str,
        #    'raw':[[x,y,z,margin], ...] (median 으로 안정 추정)}
        self.tags = {}

        # ── 입력 ────────────────────────────────────────────────
        det_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        info_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(AprilTagDetectionArray, '/apriltag_front/detections',
                                 lambda m: self.det_cb(m, 'front'), det_qos)
        self.create_subscription(AprilTagDetectionArray, '/apriltag_back/detections',
                                 lambda m: self.det_cb(m, 'back'), det_qos)
        self.create_subscription(CameraInfo, '/camera_front/color/camera_info',
                                 lambda m: self.info_cb(m, 'front'), info_qos)
        self.create_subscription(CameraInfo, '/camera_back/color/camera_info',
                                 lambda m: self.info_cb(m, 'back'), info_qos)
        # go 시 누적 리셋 (frontier_explorer 와 동일한 /explore/command 사용)
        self.create_subscription(String, '/explore/command', self.command_cb, 10)

        # ── 출력 ────────────────────────────────────────────────
        self.tags_pub = self.create_publisher(TagPoseArray, '/tags_in_map', 10)
        # 새 고유 태그 첫 포착 시 id 발행 (sound_player 등 이벤트 연동용)
        self.tag_new_pub = self.create_publisher(Int32, '/tag_new', 10)

        self.create_timer(1.0 / publish_rate, self.publish_tags)
        self.create_timer(save_period, self.save_json)

        self.get_logger().info(
            f'tag_collector 시작 — tag_size={self.tag_size}m, '
            f'map_frame={self.map_frame}, output={self.output_path}')

    # ── 콜백 ────────────────────────────────────────────────────
    def info_cb(self, msg: CameraInfo, source: str):
        if source not in self.cam_K:
            self.get_logger().info(f'[{source}] camera_info 수신 — solvePnP 가능')
        self.cam_K[source] = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.cam_D[source] = np.array(msg.d, dtype=np.float64)

    def command_cb(self, msg: String):
        if msg.data.strip().lower() == 'reset':
            n = len(self.tags)
            self.tags.clear()
            self.get_logger().info(f'reset 수신 — 태그 누적 리셋 (이전 {n}개 폐기)')

    def det_cb(self, msg: AprilTagDetectionArray, source: str):
        if cv2 is None or source not in self.cam_K:
            return  # intrinsics 아직 / cv2 없음
        optical_frame = msg.header.frame_id
        if not optical_frame:
            return
        K = self.cam_K[source]
        D = self.cam_D[source]

        for det in msg.detections:
            if det.decision_margin < self.min_margin:
                continue
            img_points = np.array([[c.x, c.y] for c in det.corners], dtype=np.float64)
            if img_points.shape != (4, 2):
                continue
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    self.object_points, img_points, K, D,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
            except cv2.error:
                continue
            if not ok:
                continue
            z = float(tvec[2])           # optical frame: +z = 렌즈 정면 거리
            if z <= 0.0 or z > self.max_range:
                continue

            # optical frame 점 → map 변환
            mp = self.to_map(tvec, optical_frame, msg.header.stamp)
            if mp is None:
                continue
            self.accumulate(int(det.id), mp, source, msg.header.stamp,
                            float(det.decision_margin))

    # ── 변환/누적 ───────────────────────────────────────────────
    def to_map(self, tvec, optical_frame, stamp):
        ps = PointStamped()
        ps.header.frame_id = optical_frame
        ps.point.x = float(tvec[0])
        ps.point.y = float(tvec[1])
        ps.point.z = float(tvec[2])
        # 1순위: 검출 stamp 시점 TF 로 변환 (주행 중 로봇 자세변화에 의한 드리프트 방지).
        #         tf_timeout 만큼 버퍼가 해당 시각 TF 를 받을 때까지 대기.
        if not self.use_latest_tf:
            ps.header.stamp = stamp
            try:
                out = self.tf_buffer.transform(
                    ps, self.map_frame,
                    timeout=Duration(seconds=self.tf_timeout))
                return [out.point.x, out.point.y, out.point.z]
            except Exception as e:   # ExtrapolationException 등 = 젯슨↔miniPC 시간 비동기
                self.get_logger().warn(
                    f'stamp TF {optical_frame}→{self.map_frame} 실패({e}) → 최신 TF 폴백',
                    throttle_duration_sec=5.0)
        # 폴백 / use_latest_tf=True: stamp=0 → tf2 가 "최신" TF 사용
        ps.header.stamp = TimeMsg()
        try:
            out = self.tf_buffer.transform(ps, self.map_frame)
            return [out.point.x, out.point.y, out.point.z]
        except Exception as e:   # LookupException 등
            self.get_logger().warn(
                f'TF {optical_frame}→{self.map_frame} 실패: {e}',
                throttle_duration_sec=5.0)
            return None

    def accumulate(self, tag_id, p, source, stamp, margin):
        t = self.tags.get(tag_id)
        if t is None:
            self.tags[tag_id] = {
                'count': 1, 'last_seen': stamp,
                'source': source, 'raw': [[p[0], p[1], p[2], margin]],
            }
            self.get_logger().info(
                f'태그 #{tag_id} 첫 관측 [{source}] map=({p[0]:.2f},{p[1]:.2f})')
            self.tag_new_pub.publish(Int32(data=tag_id))   # 사운드 등 이벤트
            return
        t['count'] += 1
        t['last_seen'] = stamp
        t['source'] = source
        t['raw'].append([p[0], p[1], p[2], margin])
        if len(t['raw']) > self.raw_cap:        # 메모리 상한 (오래된 것 버림)
            t['raw'].pop(0)

    @staticmethod
    def _estimate(t):
        """누적 원시관측의 median 위치 (EMA 보다 드리프트/이상치에 강건)."""
        arr = np.asarray(t['raw'], dtype=np.float64)[:, :3]
        return np.median(arr, axis=0)

    # ── 출력 ────────────────────────────────────────────────────
    def publish_tags(self):
        if not self.tags:
            return
        arr = TagPoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = self.map_frame
        for tag_id, t in sorted(self.tags.items()):
            est = self._estimate(t)
            tp = TagPose()
            tp.header = arr.header
            tp.tag_id = tag_id
            tp.pose.pose.position.x = float(est[0])
            tp.pose.pose.position.y = float(est[1])
            tp.pose.pose.position.z = float(est[2])
            tp.pose.pose.orientation.w = 1.0   # 자세 미사용(2D)
            tp.detection_confidence = min(1.0, t['count'] / 10.0)
            tp.last_seen = t['last_seen']
            tp.source_camera = t['source']
            tp.observation_count = t['count']
            arr.tags.append(tp)
        self.tags_pub.publish(arr)

    def save_json(self):
        if not self.tags:
            return
        data = {
            'map_frame': self.map_frame,
            'tag_size': self.tag_size,
            'tags': {
                str(tag_id): {
                    'estimate_median': self._estimate(t).tolist(),
                    'count': t['count'],
                    'source': t['source'],
                    'observations': t['raw'],   # [x,y,z,margin] — 3단계 DBSCAN 입력
                } for tag_id, t in self.tags.items()
            },
        }
        try:
            tmp = self.output_path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, self.output_path)   # 원자적 교체
        except OSError as e:
            self.get_logger().warn(f'tag_observations 저장 실패: {e}',
                                   throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = TagCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_json()   # 종료 시 마지막 저장
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
