"""
safety_layer.py — 최종 cmd_vel 안전 게이트 (jackal_mine_detection 에서 2D 스택으로 이관).

상위 노드(pure_pursuit/mission)가 발행하는 /cmd_vel_raw 를 받아 안전 필터를 거쳐
실제 cmd_vel_topic(/j100_0915/cmd_vel)으로 중계한다. 상위가 폭주해도 이 레이어가
속도 상한·충돌 직전 정지를 보장한다.

★ FAST-LIO 스택 → 2D(slam_toolbox) 스택 이관 시 바뀐 점:
  - 근접 정지 입력: /livox/lidar(CustomMsg, livox_frame) → /scan(LaserScan, base_link)
    · /scan 은 pointcloud_to_laserscan 이 이미 base_link 로 변환 + z 슬라이스(0.2~0.6m)
      + range_min 0.45 로 본체 제외한 결과. 따라서 거리 임계는 "base_link 원점 기준"이며
      livox_frame 시절 extent 보정(front 0.15/rear 0.45)은 폐기하고 직접 임계로 둔다.
    · ⚠ front_stop_dist/back_stop_dist 는 PLACEHOLDER — 실기에서 재측정/재검증 필수.
      (옛 값은 다른 프레임·센서표현 기준이라 그대로 신뢰 불가)
  - 끼임 감지 odom: /Odometry(FAST-LIO) → /j100_0915/platform/odom(휠오돔).
    휠오돔이 스캔매칭 점프가 없어 끼임 판정에 오히려 적합.

기능:
  1. 긴급 pause: /pause(Bool) true→즉시 0 유지, false→재개. (twist_mux joy(10)>external(1)
     이므로 패드 수동 개입은 항상 가능)
  2. 속도 상한 clamp.
  3. 근접 정지: 진행방향 swept-path 내 base_link 기준 stop_dist 이내 장애물 → linear 0
     (회전은 허용 — 탈출 가능해야 함).
  4. watchdog: /cmd_vel_raw 끊기면 발행 중지(twist_mux timeout 0.5s 가 최종 정지).
  5. 끼임 자동 정지/복구: 이동명령 지속에도 휠오돔 무이동이면 여유 방향 저속 탈출,
     소진 시 pause 래치.
  6. 미션 완료 정지: /final_goal_reached 수신 시 자동 pause 래치.

입력:  /cmd_vel_raw(TwistStamped), /scan(LaserScan, BEST_EFFORT), /pause(Bool),
       odom_topic(Odometry, 끼임용), /final_goal_reached(Bool)
출력:  cmd_vel_topic(TwistStamped), /safety/state(String, 1Hz), /pause(전파)
"""
from __future__ import annotations
import math
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class SafetyLayerNode(Node):
    def __init__(self) -> None:
        super().__init__('safety_layer')

        self.declare_parameter('raw_topic',      '/cmd_vel_raw')
        self.declare_parameter('cmd_vel_topic',  'cmd_vel')
        self.declare_parameter('scan_topic',     '/scan')
        self.declare_parameter('pause_topic',    '/pause')
        self.declare_parameter('base_frame',     'base_link')

        self.declare_parameter('max_linear',     0.5)    # m/s
        self.declare_parameter('max_angular',    0.5)    # rad/s
        # ⚠ PLACEHOLDER — base_link 원점→장애물 거리 임계. 실기 재검증 필수.
        #   /scan range_min(0.45) 보다 커야 의미 있음(그 안쪽은 안 보임).
        self.declare_parameter('front_stop_dist', 0.50)  # m
        self.declare_parameter('back_stop_dist',  0.50)  # m
        # swept-path 반폭: 차폭 0.47/2 + 여유. reactive/pure_pursuit 와 동기 유지.
        self.declare_parameter('swept_half_width', 0.235)  # m
        self.declare_parameter('raw_timeout_sec', 0.5)
        self.declare_parameter('scan_timeout_sec', 1.0)  # /scan 끊기면 전·후진 차단
        self.declare_parameter('start_paused', False)
        # ── 끼임(stuck) 자동 정지 ────────────────────────────────────
        self.declare_parameter('odom_topic',        '/j100_0915/platform/odom')
        self.declare_parameter('stuck_enable',      True)
        self.declare_parameter('stuck_timeout_sec', 6.0)
        self.declare_parameter('stuck_min_disp_m',  0.06)
        self.declare_parameter('stuck_min_yaw_rad', 0.10)
        self.declare_parameter('stuck_lin_thresh',  0.05)
        self.declare_parameter('stuck_ang_thresh',  0.10)
        self.declare_parameter('stuck_recovery_max',    2)
        self.declare_parameter('stuck_recovery_time',   2.0)
        self.declare_parameter('stuck_recovery_speed',  0.10)
        self.declare_parameter('stuck_recovery_window', 60.0)

        self._raw_topic   = str(self.get_parameter('raw_topic').value)
        self._cmd_topic   = str(self.get_parameter('cmd_vel_topic').value)
        self._scan_topic  = str(self.get_parameter('scan_topic').value)
        self._pause_topic = str(self.get_parameter('pause_topic').value)
        self._base_frame  = str(self.get_parameter('base_frame').value)
        self._max_lin     = float(self.get_parameter('max_linear').value)
        self._max_ang     = float(self.get_parameter('max_angular').value)
        self._front_stop  = float(self.get_parameter('front_stop_dist').value)
        self._back_stop   = float(self.get_parameter('back_stop_dist').value)
        self._swept_half  = float(self.get_parameter('swept_half_width').value)
        self._raw_to      = float(self.get_parameter('raw_timeout_sec').value)
        self._scan_to     = float(self.get_parameter('scan_timeout_sec').value)
        self._odom_topic  = str(self.get_parameter('odom_topic').value)
        self._stuck_on    = bool(self.get_parameter('stuck_enable').value)
        self._stuck_to    = float(self.get_parameter('stuck_timeout_sec').value)
        self._stuck_disp  = float(self.get_parameter('stuck_min_disp_m').value)
        self._stuck_yaw   = float(self.get_parameter('stuck_min_yaw_rad').value)
        self._stuck_lin   = float(self.get_parameter('stuck_lin_thresh').value)
        self._stuck_ang   = float(self.get_parameter('stuck_ang_thresh').value)
        self._rec_max     = int(self.get_parameter('stuck_recovery_max').value)
        self._rec_time    = float(self.get_parameter('stuck_recovery_time').value)
        self._rec_speed   = float(self.get_parameter('stuck_recovery_speed').value)
        self._rec_window  = float(self.get_parameter('stuck_recovery_window').value)

        # ── 상태 ─────────────────────────────────────────────────────
        self._paused = bool(self.get_parameter('start_paused').value)
        if self._paused:
            self.get_logger().warn('start_paused=true — /pause false 수신 전까지 정지 유지')
        self._front_min: float = float('inf')
        self._back_min:  float = float('inf')
        self._last_scan_t: Optional[float] = None
        self._last_raw_t:  Optional[float] = None
        self._odom_hist: Deque[Tuple[float, float, float, float]] = deque()
        self._cmd_motion_since: Optional[float] = None
        self._recover_until: Optional[float] = None
        self._recover_dir = -1.0
        self._stuck_times: Deque[float] = deque()
        self._n_pass = self._n_clamp = self._n_block = self._n_stuck = 0

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )

        self.create_subscription(TwistStamped, self._raw_topic, self._on_raw, 10)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, scan_qos)
        self.create_subscription(Bool, self._pause_topic, self._on_pause, 10)
        if self._stuck_on:
            self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(Bool, '/final_goal_reached', self._on_final, 10)
        self._pub = self.create_publisher(TwistStamped, self._cmd_topic, 10)
        self._dbg = self.create_publisher(String, '/safety/state', 10)
        self._pause_pub = self.create_publisher(Bool, self._pause_topic, 10)

        self.create_timer(0.1, self._pause_tick)
        self.create_timer(1.0, self._publish_debug)

        self.get_logger().info(
            f'safety_layer up: {self._raw_topic} → {self._cmd_topic}, '
            f'clamp lin≤{self._max_lin} ang≤{self._max_ang}, '
            f'stop front<{self._front_stop}m back<{self._back_stop}m (base_link), '
            f'scan={self._scan_topic}, odom={self._odom_topic}'
        )

    # ── 콜백: pause ──────────────────────────────────────────────────
    def _on_pause(self, msg: Bool) -> None:
        if msg.data != self._paused:
            self.get_logger().warn(
                '⛔ PAUSE — 즉시 정지' if msg.data else '▶ RESUME — 주행 재개')
        self._paused = bool(msg.data)
        if self._paused:
            self._publish(0.0, 0.0)

    def _pause_tick(self) -> None:
        if self._paused:
            self._publish(0.0, 0.0)
            return
        if self._recover_until is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            clear = (self._front_min - self._front_stop if self._recover_dir > 0
                     else self._back_min - self._back_stop)
            if now >= self._recover_until or clear < 0.05:
                self._recover_until = None
                self._cmd_motion_since = None
                self.get_logger().info('복구 기동 종료 — 정상 재개')
            else:
                self._publish(self._recover_dir * self._rec_speed, 0.0)

    def _on_final(self, msg: Bool) -> None:
        if msg.data and not self._paused:
            self._paused = True
            self.get_logger().warn('🏁 미션 완료 (/final_goal_reached) — 자동 정지. 재가동: /pause false')
            self._publish(0.0, 0.0)

    # ── 콜백: LaserScan(base_link) — 진행방향 최근접 거리 ────────────
    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan_t = self.get_clock().now().nanoseconds * 1e-9
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = ranges.size
        if n == 0:
            return
        ang = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        rmax = msg.range_max if msg.range_max > 0 else np.inf
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= rmax)
        if not valid.any():
            self._front_min = self._back_min = float('inf')
            return
        r = ranges[valid]; a = ang[valid]
        x = r * np.cos(a)   # base_link: +x 정면
        y = r * np.sin(a)   # +y 좌
        band = np.abs(y) < self._swept_half
        fwd = band & (x > 0.0)
        bwd = band & (x < 0.0)
        self._front_min = float(x[fwd].min())     if fwd.any() else float('inf')
        self._back_min  = float((-x[bwd]).min())  if bwd.any() else float('inf')

    # ── 콜백: Odometry(휠오돔) — 끼임 감지 포즈 이력 ────────────────
    def _on_odom(self, msg: Odometry) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom_hist.append((t, p.x, p.y, yaw))
        cutoff = t - (self._stuck_to + 2.0)
        while self._odom_hist and self._odom_hist[0][0] < cutoff:
            self._odom_hist.popleft()

    def _stuck_check(self, now: float, lin: float, ang: float) -> None:
        commanding = abs(lin) >= self._stuck_lin or abs(ang) >= self._stuck_ang
        if not commanding:
            self._cmd_motion_since = None
            return
        if self._cmd_motion_since is None:
            self._cmd_motion_since = now
            return
        if now - self._cmd_motion_since < self._stuck_to:
            return
        if not self._odom_hist:
            return
        old = None
        for entry in self._odom_hist:
            if now - entry[0] <= self._stuck_to:
                old = entry
                break
        cur = self._odom_hist[-1]
        if old is None or cur[0] - old[0] < self._stuck_to * 0.7:
            return
        disp = math.hypot(cur[1] - old[1], cur[2] - old[2])
        dyaw = abs(math.atan2(math.sin(cur[3] - old[3]), math.cos(cur[3] - old[3])))
        if disp < self._stuck_disp and dyaw < self._stuck_yaw:
            self._n_stuck += 1
            self._cmd_motion_since = None
            while self._stuck_times and now - self._stuck_times[0] > self._rec_window:
                self._stuck_times.popleft()
            self._stuck_times.append(now)
            scan_ok = (self._last_scan_t is not None and
                       now - self._last_scan_t < self._scan_to)
            clear_f = self._front_min - self._front_stop
            clear_b = self._back_min - self._back_stop
            if (len(self._stuck_times) <= self._rec_max and scan_ok
                    and max(clear_f, clear_b) > 0.10):
                self._recover_dir = 1.0 if clear_f > clear_b else -1.0
                self._recover_until = now + self._rec_time
                self.get_logger().warn(
                    f'🔄 STUCK {len(self._stuck_times)}/{self._rec_max} — '
                    f'{"전진" if self._recover_dir > 0 else "후진"} 복구 기동 '
                    f'({self._rec_time:.0f}s, 여유 f={clear_f:.2f}/b={clear_b:.2f})')
            else:
                self._paused = True
                self.get_logger().error(
                    f'🚨 STUCK 자동 정지 (복구 {len(self._stuck_times) - 1}회 실패) — '
                    f'{self._stuck_to:.0f}s간 명령(lin={lin:.2f}, ang={ang:.2f})에도 '
                    f'이동 {disp * 100:.1f}cm / 회전 {math.degrees(dyaw):.1f}° 뿐. '
                    f'재개: /pause false (resume)')
                m = Bool(); m.data = True
                self._pause_pub.publish(m)
                self._publish(0.0, 0.0)

    # ── 콜백: raw cmd → 필터 → 발행 ─────────────────────────────────
    def _on_raw(self, msg: TwistStamped) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self._last_raw_t = now
        if self._paused:
            return
        if self._recover_until is not None:
            return

        lin = float(msg.twist.linear.x)
        ang = float(msg.twist.angular.z)
        lin_c = max(-self._max_lin, min(self._max_lin, lin))
        ang_c = max(-self._max_ang, min(self._max_ang, ang))
        if lin_c != lin or ang_c != ang:
            self._n_clamp += 1

        scan_ok = (self._last_scan_t is not None and
                   now - self._last_scan_t < self._scan_to)
        if not scan_ok:
            if lin_c != 0.0:
                self._n_block += 1
                lin_c = 0.0
        elif lin_c > 0.0 and self._front_min < self._front_stop:
            self._n_block += 1
            lin_c = 0.0
        elif lin_c < 0.0 and self._back_min < self._back_stop:
            self._n_block += 1
            lin_c = 0.0
        else:
            self._n_pass += 1

        self._publish(lin_c, ang_c)
        if self._stuck_on:
            self._stuck_check(now, lin_c, ang_c)

    def _publish(self, lin: float, ang: float) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._base_frame
        cmd.twist.linear.x = float(lin)
        cmd.twist.angular.z = float(ang)
        self._pub.publish(cmd)

    def _publish_debug(self) -> None:
        m = String()
        m.data = (f'paused={self._paused} front={self._front_min:.2f} '
                  f'back={self._back_min:.2f} pass={self._n_pass} '
                  f'clamp={self._n_clamp} block={self._n_block} stuck={self._n_stuck}')
        self._dbg.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyLayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
