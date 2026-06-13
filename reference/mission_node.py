"""
mission_node.py — 시각 서보잉 미션 FSM (claude.md §0.9 / §0.11d-1)
==========================================================================
SCANNING 중 Jetson YOLO가 보낸 /yolo/tag_candidate(conf≥trigger_conf)를 받으면
reactive_explorer를 멈추고(상태 발행으로 게이팅, §5.3) 태그를 향해 시각 서보잉
으로 접근한다. 도착 판정은 해당 방향 카메라의 /apriltag_*/detections 가
arrival_frames 프레임 연속 수신될 때. 도착하면 **로봇의 현재 map 위치 + 진행
방향 dock_offset_m 앞 지점**을 태그 좌표로 기록한다 (도킹 위치 기록 방식 —
detections 에 pose 가 없으므로(양 distro 공통) 사용자 확정 2026-06-04).

back 카메라 검출(bearing≈±π)은 **후진으로 접근** (사용자 확정 2026-06-04).

상태 전이:
  IDLE → (auto_start) → SCANNING
  SCANNING → conf≥trigger_conf 인 candidate → APPROACHING_TAG
  APPROACHING_TAG → 도착 판정 → 기록 → COOLDOWN
                  → candidate 끊김 lost_timeout 초과 → COOLDOWN (기록 없음)
                  → max_approach_sec 초과 → COOLDOWN (기록 없음)
  COOLDOWN → cooldown_sec 경과 → SCANNING
  (모든 상태) → /pause true → PAUSED → /pause false → SCANNING
  (SCAN_DONE → CLUSTERING → APPROACH_CENTROID 는 후속 작업)

긴급 pause: /pause (std_msgs/Bool) — safety_layer 와 같은 토픽을 공유.
safety_layer 가 cmd 차단(1차), mission 은 PAUSED 로 FSM 동결(2차).
PAUSED 는 explorer active_states 밖이므로 explorer 도 자동 정지.

reactive_explorer 게이팅(§5.3): explorer 는 /mission/state 가
'SCANNING'/'COOLDOWN'/미수신일 때만 cmd_vel 발행. COOLDOWN 에 explorer 주행을
허용하는 이유: YOLO candidate 에는 tag id 가 없어 같은 태그가 재트리거되므로,
cooldown 동안 explorer 가 그 자리를 벗어나야 함.

구독:
  /yolo/tag_candidate          custom_msgs/TagCandidate
  /apriltag_front/detections   apriltag_msgs/AprilTagDetectionArray
  /apriltag_back/detections    apriltag_msgs/AprilTagDetectionArray
  TF map → base_link           (도킹 위치 기록)

발행:
  cmd_vel_topic    geometry_msgs/TwistStamped   (APPROACHING_TAG 중 20 Hz)
  /mission/state   std_msgs/String              (10 Hz, explorer 게이팅용)
  /mine_positions  geometry_msgs/PoseArray      (기록 누적, frame=map)
"""
from __future__ import annotations
import csv
import re
import math
import os
from collections import Counter
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_ros

from custom_msgs.msg import TagCandidate
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TwistStamped, PoseArray, Pose
from std_msgs.msg import String, Bool


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_node')

        # ── 토픽/프레임 ──────────────────────────────────────────────
        self.declare_parameter('candidate_topic', '/yolo/tag_candidate')
        self.declare_parameter('front_detections_topic', '/apriltag_front/detections')
        self.declare_parameter('back_detections_topic', '/apriltag_back/detections')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_raw')  # safety_layer 경유
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # ── 트리거/접근 ──────────────────────────────────────────────
        self.declare_parameter('auto_start', True)
        self.declare_parameter('trigger_conf', 0.55)   # SCANNING→APPROACHING 트리거
        self.declare_parameter('approach_speed', 0.12) # m/s (전진/후진 공통 크기)
        self.declare_parameter('k_angular', 0.8)       # rad/s per rad bearing err
        self.declare_parameter('max_angular', 0.4)     # rad/s (§8.4 FAST-LIO 한도)
        self.declare_parameter('slow_zone_rad', 0.6)   # |err|≥이값이면 직진속도 0
        self.declare_parameter('lost_timeout_sec', 2.0)
        self.declare_parameter('max_approach_sec', 60.0)

        # ── 도착/기록 ────────────────────────────────────────────────
        self.declare_parameter('arrival_frames', 8)    # 연속 non-empty detections
        self.declare_parameter('dock_offset_m', 0.5)   # 로봇→태그 추정 잔여 거리
        self.declare_parameter('cooldown_sec', 15.0)
        # 접근 방향 막힘 포기 (2026-06-05): /safety/state 의 front/back 거리로
        # 진행 방향이 blocked_abort_sec 연속 막혀 있으면 접근 포기 → COOLDOWN.
        # (APPROACHING_TAG 중엔 reactive 의 ESCAPE 가 게이트로 꺼져 있어
        #  mission 이 벽에 대고 미는 것을 막을 장치가 없었음 — stuck 2회 실측)
        self.declare_parameter('blocked_abort_sec', 1.5)
        self.declare_parameter('blocked_front_dist', 0.40)  # front 접근 시 차단 판정
        self.declare_parameter('blocked_back_dist',  0.80)  # back(후진) 접근 시 차단 판정
        self.declare_parameter('tf_timeout_sec', 0.3)
        self.declare_parameter('csv_path', '/tmp/mission_tag_positions.csv')
        self.declare_parameter('publish_rate', 2.0)    # /mine_positions Hz

        self._cand_topic  = str(self.get_parameter('candidate_topic').value)
        self._front_topic = str(self.get_parameter('front_detections_topic').value)
        self._back_topic  = str(self.get_parameter('back_detections_topic').value)
        self._cmd_topic   = str(self.get_parameter('cmd_vel_topic').value)
        self._map_f       = str(self.get_parameter('map_frame').value)
        self._base_f      = str(self.get_parameter('base_frame').value)
        self._auto_start  = bool(self.get_parameter('auto_start').value)
        self._trig_conf   = float(self.get_parameter('trigger_conf').value)
        self._v_app       = float(self.get_parameter('approach_speed').value)
        self._k_ang       = float(self.get_parameter('k_angular').value)
        self._max_ang     = float(self.get_parameter('max_angular').value)
        self._slow_zone   = float(self.get_parameter('slow_zone_rad').value)
        self._lost_to     = float(self.get_parameter('lost_timeout_sec').value)
        self._blk_abort   = float(self.get_parameter('blocked_abort_sec').value)
        self._blk_front   = float(self.get_parameter('blocked_front_dist').value)
        self._blk_back    = float(self.get_parameter('blocked_back_dist').value)
        self._max_app     = float(self.get_parameter('max_approach_sec').value)
        self._arr_frames  = int(self.get_parameter('arrival_frames').value)
        self._dock_off    = float(self.get_parameter('dock_offset_m').value)
        self._cooldown    = float(self.get_parameter('cooldown_sec').value)
        self._tf_to       = float(self.get_parameter('tf_timeout_sec').value)
        self._csv_path    = str(self.get_parameter('csv_path').value)
        pub_rate          = float(self.get_parameter('publish_rate').value)

        # ── TF2 ──────────────────────────────────────────────────────
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        # ── 상태 ─────────────────────────────────────────────────────
        self._state = 'SCANNING' if self._auto_start else 'IDLE'
        self._state_since = self.get_clock().now()

        self._last_cand: Optional[TagCandidate] = None
        self._safety_front = float('inf')   # /safety/state 파싱 (접근 방향 막힘 판정)
        self._safety_back = float('inf')
        self._blocked_since: Optional[Time] = None
        self._last_cand_time: Optional[Time] = None
        self._approach_cam = 'front'        # 이번 접근에 쓰는 카메라
        self._arrival_count = 0             # 연속 non-empty detections 카운트
        self._arrival_ids: list[int] = []   # 도착 윈도 동안 보인 id 들
        self._last_seen_ids: dict[int, Time] = {}  # 최근 detections id → 시각

        # 기록 저장소: tag_id → (x, y, z)
        self._recorded: dict[int, tuple[float, float, float]] = {}

        self._init_csv()

        # ── 구독/발행 ────────────────────────────────────────────────
        self.declare_parameter('pause_topic', '/pause')
        self.create_subscription(
            Bool, str(self.get_parameter('pause_topic').value), self._on_pause, 10)
        self.create_subscription(TagCandidate, self._cand_topic, self._on_candidate, 10)
        self.create_subscription(String, '/safety/state', self._on_safety_state, 10)
        self.create_subscription(
            AprilTagDetectionArray, self._front_topic,
            lambda m: self._on_detections(m, 'front'), 10)
        self.create_subscription(
            AprilTagDetectionArray, self._back_topic,
            lambda m: self._on_detections(m, 'back'), 10)

        self._pub_cmd   = self.create_publisher(TwistStamped, self._cmd_topic, 10)
        self._pub_state = self.create_publisher(String, '/mission/state', 10)
        self._pub_pos   = self.create_publisher(PoseArray, '/mine_positions', 10)

        # ── 타이머 ───────────────────────────────────────────────────
        self.create_timer(0.05, self._control_tick)       # 20 Hz 제어
        self.create_timer(0.1,  self._publish_state)      # 10 Hz 상태
        self.create_timer(1.0 / pub_rate, self._publish_positions)
        self.create_timer(2.0,  self._log_status)

        self.get_logger().info(
            f'mission_node up: state={self._state}, cand={self._cand_topic}, '
            f'cmd={self._cmd_topic}, trig_conf={self._trig_conf}, '
            f'v={self._v_app}, dock_off={self._dock_off}m, '
            f'back 접근=후진'
        )

    # ── 상태 전이 ─────────────────────────────────────────────────────
    def _transition(self, new_state: str, reason: str) -> None:
        self.get_logger().info(f'[mission] {self._state} → {new_state} ({reason})')
        self._state = new_state
        self._state_since = self.get_clock().now()
        if new_state == 'APPROACHING_TAG':
            self._arrival_count = 0
            self._arrival_ids = []

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._state_since).nanoseconds * 1e-9

    # ── 콜백: 긴급 pause ─────────────────────────────────────────────
    def _on_pause(self, msg: Bool) -> None:
        if msg.data and self._state != 'PAUSED':
            self._publish_cmd(0.0, 0.0)
            self._transition('PAUSED', '/pause 수신 — 긴급 정지')
        elif not msg.data and self._state == 'PAUSED':
            self._transition('SCANNING', '/pause 해제 — 재개')

    # ── 콜백: YOLO candidate ─────────────────────────────────────────
    def _on_candidate(self, msg: TagCandidate) -> None:
        self._last_cand = msg
        self._last_cand_time = self.get_clock().now()

        if self._state != 'SCANNING':
            return
        if msg.confidence < self._trig_conf:
            return

        # 최근(2s) detections 의 id 가 전부 기록 완료면 무시 (YOLO 재트리거 억제.
        # 단 YOLO 만 보이고 apriltag 은 아직 못 잡는 원거리에선 id 미상 → 접근 진행)
        now = self.get_clock().now()
        recent = {tid for tid, t in self._last_seen_ids.items()
                  if (now - t).nanoseconds * 1e-9 < 2.0}
        if recent and recent.issubset(self._recorded.keys()):
            return

        self._approach_cam = msg.source_camera if msg.source_camera in ('front', 'back') else 'front'
        self._transition('APPROACHING_TAG',
                         f'candidate conf={msg.confidence:.2f} cam={self._approach_cam} '
                         f'bearing={math.degrees(msg.bearing_rad):.0f}°')

    # ── 콜백: AprilTag detections ────────────────────────────────────
    def _on_detections(self, msg: AprilTagDetectionArray, cam: str) -> None:
        now = self.get_clock().now()
        ids = [int(d.id) for d in msg.detections]
        for tid in ids:
            self._last_seen_ids[tid] = now

        if self._state != 'APPROACHING_TAG' or cam != self._approach_cam:
            return

        # 도착 판정: 접근 방향 카메라의 detections 연속 수신
        if ids:
            self._arrival_count += 1
            self._arrival_ids.extend(ids)
        else:
            self._arrival_count = 0
            self._arrival_ids = []

        if self._arrival_count >= self._arr_frames:
            self._record_and_finish()

    # ── 도착 → 기록 ──────────────────────────────────────────────────
    def _record_and_finish(self) -> None:
        try:
            tr = self._tf_buf.lookup_transform(
                self._map_f, self._base_f, Time(),
                timeout=Duration(seconds=self._tf_to))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'[mission] 도착했으나 TF({self._map_f}←{self._base_f}) 실패: {e}')
            self._transition('COOLDOWN', 'TF 실패로 기록 불가')
            return

        rx = tr.transform.translation.x
        ry = tr.transform.translation.y
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # 태그 위치 추정 = 로봇 위치 + 진행방향(전진=heading, 후진=반대) dock_offset
        direction = yaw if self._approach_cam == 'front' else yaw + math.pi
        tx = rx + self._dock_off * math.cos(direction)
        ty = ry + self._dock_off * math.sin(direction)

        tag_id = Counter(self._arrival_ids).most_common(1)[0][0]

        if tag_id in self._recorded:
            self.get_logger().info(f'[mission] tag {tag_id} 이미 기록됨 — 스킵')
        else:
            self._recorded[tag_id] = (tx, ty, 0.0)
            self._append_csv(tag_id, tx, ty)
            self.get_logger().info(
                f'[mission] tag {tag_id} 기록 → map ({tx:.3f}, {ty:.3f}) '
                f'[cam={self._approach_cam}, robot=({rx:.2f},{ry:.2f}), yaw={math.degrees(yaw):.0f}°]')

        self._publish_positions()
        self._transition('COOLDOWN', f'tag {tag_id} 도킹 완료')

    # ── 20 Hz 제어 ───────────────────────────────────────────────────
    def _control_tick(self) -> None:
        if self._state == 'COOLDOWN':
            if self._elapsed() >= self._cooldown:
                self._transition('SCANNING', 'cooldown 종료')
            return
        if self._state != 'APPROACHING_TAG':
            return

        now = self.get_clock().now()

        # 이탈 조건
        if self._elapsed() > self._max_app:
            self._publish_cmd(0.0, 0.0)
            self._transition('COOLDOWN', f'접근 타임아웃 {self._max_app}s')
            return
        if (self._last_cand_time is None or
                (now - self._last_cand_time).nanoseconds * 1e-9 > self._lost_to):
            self._publish_cmd(0.0, 0.0)
            self._transition('COOLDOWN', 'candidate 끊김 (lost)')
            return

        # 접근 방향 막힘 → 포기 (벽에 대고 밀기 방지, 2026-06-05)
        blocked = (self._safety_front < self._blk_front
                   if self._approach_cam == 'front'
                   else self._safety_back < self._blk_back)
        if blocked:
            if self._blocked_since is None:
                self._blocked_since = now
            elif (now - self._blocked_since).nanoseconds * 1e-9 > self._blk_abort:
                self._publish_cmd(0.0, 0.0)
                self._blocked_since = None
                self._transition('COOLDOWN',
                                 f'접근 방향({self._approach_cam}) 막힘 — 포기')
                return
        else:
            self._blocked_since = None

        cand = self._last_cand
        # 접근 중 카메라가 바뀌면 (ex. 지나쳐서 반대 카메라에 잡힘) 따라감
        if cand.source_camera in ('front', 'back'):
            self._approach_cam = cand.source_camera

        # 서보잉: front=bearing→0 전진 / back=bearing→π 후진
        if self._approach_cam == 'front':
            err = _wrap(cand.bearing_rad)
            sign = 1.0
        else:
            err = _wrap(cand.bearing_rad - math.pi)
            sign = -1.0

        ang = max(-self._max_ang, min(self._max_ang, self._k_ang * err))
        lin = sign * self._v_app * max(0.0, 1.0 - abs(err) / self._slow_zone)
        self._publish_cmd(lin, ang)

    # ── 콜백: /safety/state 파싱 (front/back 최근접 거리) ─────────────
    def _on_safety_state(self, msg: String) -> None:
        m = re.search(r'front=([\d.inf]+) back=([\d.inf]+)', msg.data)
        if m:
            try:
                self._safety_front = float(m.group(1))
                self._safety_back = float(m.group(2))
            except ValueError:
                pass

    # ── 발행 ─────────────────────────────────────────────────────────
    def _publish_cmd(self, lin: float, ang: float) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._base_f
        cmd.twist.linear.x = float(lin)
        cmd.twist.angular.z = float(ang)
        self._pub_cmd.publish(cmd)

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._state
        self._pub_state.publish(msg)

    def _publish_positions(self) -> None:
        if not self._recorded:
            return
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = self._map_f
        for x, y, z in self._recorded.values():
            p = Pose()
            p.position.x, p.position.y, p.position.z = x, y, z
            p.orientation.w = 1.0
            pa.poses.append(p)
        self._pub_pos.publish(pa)

    def _log_status(self) -> None:
        if self._state == 'APPROACHING_TAG' and self._last_cand is not None:
            self.get_logger().info(
                f'[mission] APPROACHING cam={self._approach_cam} '
                f'bearing={math.degrees(self._last_cand.bearing_rad):.0f}° '
                f'conf={self._last_cand.confidence:.2f} '
                f'arrival={self._arrival_count}/{self._arr_frames}',
                throttle_duration_sec=2.0)

    # ── CSV ──────────────────────────────────────────────────────────
    def _init_csv(self) -> None:
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self._csv_path)), exist_ok=True)
            with open(self._csv_path, 'w', newline='') as f:
                csv.writer(f).writerow(['tag_id', 'x', 'y', 'z', 'timestamp_sec'])
        except Exception as e:
            self.get_logger().error(f'CSV 초기화 실패: {e}')

    def _append_csv(self, tag_id: int, x: float, y: float) -> None:
        try:
            with open(self._csv_path, 'a', newline='') as f:
                csv.writer(f).writerow([
                    tag_id, round(x, 4), round(y, 4), 0.0,
                    self.get_clock().now().nanoseconds // 10 ** 9])
        except Exception as e:
            self.get_logger().error(f'CSV 기록 실패: {e}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
