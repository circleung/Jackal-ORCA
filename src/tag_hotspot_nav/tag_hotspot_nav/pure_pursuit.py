"""
pure_pursuit.py — /plan 경로를 추종해 cmd_vel 을 내는 노드 (Nav2 controller 대체).

웹사이트(RBE3002) pure_pursuit.py 의 설계를 Jackal 에 맞게 적응:
  - lookahead 0.18m(터틀봇) → 0.4m (Jackal 차체·속도 스케일)
  - 출력: TwistStamped → /j100_0915/cmd_vel
    (twist_mux 우선순위 1 — 조이스틱(10)/RC(12)가 항상 이김 = 수동 오버라이드 안전)
  - /scan 전방 최소거리 기반 감속/정지 (원본의 obstacle avoidance 단순화 버전.
    경로 자체가 C-space + costmap 으로 벽을 피하므로 여기선 최후 방어만)

알고리즘:
  1. TF map→base_link 로 로봇 포즈
  2. 경로에서 로봇 앞쪽으로 lookahead 거리 이상 떨어진 첫 점 = 목표점
  3. 목표점을 로봇 좌표계로 변환 → 곡률 κ = 2y / L²  → ω = v·κ
  4. 전/후진 선택: 목표가 뒤쪽이면 180° 제자리 회전 대신 후진(스키드스티어 대칭).
     회전이 적은 쪽을 고르고, 진행축 기준 방위 오차가 크면 제자리 회전 먼저.
     (allow_reverse=False 면 항상 전진 = 기존 동작)
  5. 경로 끝 도달 → 정지 + /goal_reached 발행
  · 진행 방향 라이다 clearance 로 감속/정지 (전진=전방 ±15°, 후진=후방 ±15°).
    ⚠ 후진 cliff(낭떠러지)는 미고려 — 현 단계 범위 밖.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        # ── 파라미터 ─────────────────────────────────────────────
        self.declare_parameter('cmd_vel_topic', '/j100_0915/cmd_vel')
        self.declare_parameter('linear_speed', 0.3)       # [m/s]
        self.declare_parameter('max_angular', 1.0)        # [rad/s]
        self.declare_parameter('lookahead', 0.4)          # [m]
        self.declare_parameter('goal_tolerance', 0.15)    # [m]
        self.declare_parameter('rotate_in_place_angle', 1.0)   # [rad] 이 이상 틀어지면 제자리 회전
        # 회전모드 진입/해제 히스테리시스: 진입(rotate_in_place_angle) > 해제(rotate_exit_angle).
        # 둘이 같으면 heading_err 가 경계에서 노이즈로 흔들릴 때마다 모드가 매 cycle 토글되어
        # "덜덜덜" 떨림 + 정지-주행 반복이 발생 (좁은 복도 급커브에서 특히 심함).
        self.declare_parameter('rotate_exit_angle', 0.6)       # [rad] 회전모드 해제 임계 (< rotate_in_place_angle)
        # 방위오차 비례 선속도 감속의 분모. rotate_in_place_angle(=1.0)과 분리해서
        # 둬야 "주행 중 회전(0.6~1.0rad)" 구간에서 선속도가 과하게(0.4배) 안 깎인다.
        self.declare_parameter('heading_slow_angle', 1.8)      # [rad] 클수록 회전 중 선속도 유지
        self.declare_parameter('slow_down_dist', 0.8)     # [m] 전방 장애물 감속 시작
        self.declare_parameter('stop_dist', 0.35)         # [m] 전방 장애물 정지 진입
        # 정지 히스테리시스: stop_dist 에서 멈추고, stop_release_dist 이상 트여야 재출발.
        # 둘이 같으면 clearance 노이즈로 매 틱 정지↔주행 토글(=떨림/기어감)이 생긴다.
        self.declare_parameter('stop_release_dist', 0.50) # [m] 정지 해제 임계 (> stop_dist)
        self.declare_parameter('front_sector_deg', 15.0)  # [deg] 전방 감시 섹터 반각 (정지/감속 판정)
        self.declare_parameter('control_rate', 20.0)      # [Hz]
        # 회전 안전 가드: 전방위 최근접 장애물이 가까우면 회전 속도를 줄인다(멈춤 X → 교착 방지).
        # 차체가 회전 시 모서리로 주변을 쓸기 때문. ⚠ /scan range_min 아래는 라이다 사각.
        # range_min 0.20 으로 차체모서리(~0.33m) 권역까지 보이므로 임계를 그에 맞춤.
        self.declare_parameter('rotate_slow_clearance', 0.30)  # [m] 이 아래부터 회전 감속 (복도벽 0.5m 에서는 풀속)
        self.declare_parameter('rotate_stop_clearance', 0.20)  # [m] 이 이하면 최저속 회전(차체모서리 근접)
        self.declare_parameter('rotate_min_scale', 0.50)       # 회전 최저 속도 배율 — 최저 50%로 상향
        # 후진 추종: 목표가 뒤쪽이면 제자리 180° 회전 대신 후진(움직임 제한 상황에 효율적).
        self.declare_parameter('allow_reverse', True)          # 후진 허용 (False=항상 전진)
        self.declare_parameter('reverse_hysteresis', 0.3)      # [rad] 전/후진 전환 떨림 억제 여유
        # 각속도 EMA 스무딩: 웨이포인트 전환 시 각속도 급변 → 진동 억제
        self.declare_parameter('ang_smooth', 0.45)             # EMA alpha (낮을수록 강한 스무딩)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.max_angular = self.get_parameter('max_angular').value
        self.lookahead = self.get_parameter('lookahead').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.rotate_angle = self.get_parameter('rotate_in_place_angle').value
        self.rotate_exit_angle = self.get_parameter('rotate_exit_angle').value
        self.heading_slow_angle = float(self.get_parameter('heading_slow_angle').value)
        self.slow_down_dist = self.get_parameter('slow_down_dist').value
        self.stop_dist = self.get_parameter('stop_dist').value
        self.stop_release_dist = float(self.get_parameter('stop_release_dist').value)
        self.front_sector = math.radians(self.get_parameter('front_sector_deg').value)
        control_rate = self.get_parameter('control_rate').value
        self.rotate_slow_clearance = self.get_parameter('rotate_slow_clearance').value
        self.rotate_stop_clearance = self.get_parameter('rotate_stop_clearance').value
        self.rotate_min_scale = self.get_parameter('rotate_min_scale').value
        self.allow_reverse = self.get_parameter('allow_reverse').value
        self.reverse_hysteresis = self.get_parameter('reverse_hysteresis').value
        self.ang_smooth = float(self.get_parameter('ang_smooth').value)

        # ── 입출력 ──────────────────────────────────────────────
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, scan_qos)
        self.command_sub = self.create_subscription(
            String, '/explore/command', self.command_callback, 10)

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)
        # 전방 막힘 신호 (사운드 등 이벤트용)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_block', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── 상태 ────────────────────────────────────────────────
        self.path = []            # [(x, y)] map frame
        self.path_index = 0       # 진행 인덱스 (뒤로 안 돌아감)
        self.front_clearance = float('inf')
        self.rear_clearance = float('inf')       # 후방 ±15° 최근접 (후진 정지용)
        self.surround_clearance = float('inf')   # 전방위 최근접 (회전 안전 가드용)
        self._reverse = False     # 현재 후진 추종 중 여부 (히스테리시스 상태)
        self._rotating = False    # 현재 제자리회전 모드 여부 (히스테리시스 상태)
        self._stopped = False     # 전방 장애물 정지 상태 (히스테리시스)
        self.paused = False       # pause 명령 시 즉시 정지 (경로는 유지)
        self._prev_ang = 0.0      # 각속도 EMA 스무딩용 이전 값

        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        self.get_logger().info(f'pure_pursuit 시작 → {self.cmd_vel_topic}')

    # ── 콜백 ───────────────────────────────────────────────────
    def path_callback(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_index = 0
        self._reverse = False     # 새 경로 → 전/후진 상태 초기화
        self._rotating = False    # 새 경로 → 회전모드 상태 초기화
        self._stopped = False     # 새 경로 → 정지 히스테리시스 리셋
        self._prev_ang = 0.0      # 새 경로 → EMA 리셋
        if self.path:
            self.get_logger().info(f'새 경로 수신: {len(self.path)} waypoints')
        else:
            self.get_logger().info('빈 경로 수신 → 정지')
            self.publish_stop()

    def command_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'pause' and not self.paused:
            self.paused = True
            self.publish_stop()
            self.get_logger().info("명령 'pause' → 정지")
        elif cmd in ('go', 'resume', 'reset') and self.paused:
            self.paused = False
            self.get_logger().info(f"명령 '{cmd}' → 주행 재개")

    def scan_callback(self, msg: LaserScan):
        """전방 섹터(front_sector_deg) 최소(직진 정지용) + 전방위 최소(회전 안전용)."""
        n = len(msg.ranges)
        if n == 0:
            return
        front = float('inf')
        rear = float('inf')
        surround = float('inf')
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            surround = min(surround, r)        # 회전 시 차체가 쓸고 가는 전방위 최근접
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) < self.front_sector:
                front = min(front, r)
            elif abs(abs(angle) - math.pi) < math.radians(15):   # 후방 ±15° (후진 정지용)
                rear = min(rear, r)
        self.front_clearance = front
        self.rear_clearance = rear
        self.surround_clearance = surround
        self.obstacle_pub.publish(Bool(data=bool(front < self.stop_dist)))

    def _rot_scale(self) -> float:
        """주변(전방위) 장애물 거리에 따른 회전 속도 배율.
        좁을수록 천천히 회전(멈추진 않음 → 교착 방지). ⚠ /scan range_min 아래는 사각."""
        c = self.surround_clearance
        if c >= self.rotate_slow_clearance:
            return 1.0
        if c <= self.rotate_stop_clearance:
            return self.rotate_min_scale
        f = (c - self.rotate_stop_clearance) / \
            (self.rotate_slow_clearance - self.rotate_stop_clearance)
        return self.rotate_min_scale + (1.0 - self.rotate_min_scale) * f

    # ── 로봇 포즈 ──────────────────────────────────────────────
    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            tr = t.transform.translation
            return tr.x, tr.y, yaw_from_quaternion(t.transform.rotation)
        except Exception:
            return None

    # ── 제어 루프 ──────────────────────────────────────────────
    def control_loop(self):
        if self.paused or not self.path:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        # 1) 도착 판정: 마지막 waypoint 까지의 거리
        gx, gy = self.path[-1]
        if math.hypot(gx - rx, gy - ry) < self.goal_tolerance:
            self.get_logger().info('경로 끝 도달')
            self.path = []
            self.publish_stop()
            self.goal_reached_pub.publish(Bool(data=True))
            return

        # 2) lookahead 목표점: 진행 인덱스부터 앞으로만 탐색
        target = None
        for i in range(self.path_index, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - rx, py - ry) >= self.lookahead:
                target = (px, py)
                self.path_index = i
                break
        if target is None:
            target = self.path[-1]   # 전부 lookahead 안쪽 → 최종점 직접 추종

        # 3) 목표점을 로봇 좌표계로
        dx = target[0] - rx
        dy = target[1] - ry
        lx = math.cos(-ryaw) * dx - math.sin(-ryaw) * dy   # 로봇 전방 성분
        ly = math.sin(-ryaw) * dx + math.cos(-ryaw) * dy   # 로봇 좌측 성분
        bearing = math.atan2(ly, lx)

        # 4) 전/후진 선택 (스키드스티어라 후진도 대칭으로 추종 가능).
        #    목표가 뒤쪽이면 180° 제자리 회전보다 후진이 효율적·안전.
        #    회전이 더 적은 쪽을 고르되, 90° 경계 방향 떨림은 히스테리시스로 억제.
        rev_err = math.atan2(math.sin(bearing - math.pi), math.cos(bearing - math.pi))
        if self.allow_reverse:
            if self._reverse:
                if abs(bearing) + self.reverse_hysteresis < abs(rev_err):
                    self._reverse = False     # 전진이 충분히 더 나음 → 전환
            elif abs(rev_err) + self.reverse_hysteresis < abs(bearing):
                self._reverse = True          # 후진이 충분히 더 나음 → 전환
        else:
            self._reverse = False

        if self._reverse:
            direction = -1.0
            heading_err = rev_err             # 진행축(차체 뒤) 기준 방위 오차
            clearance = self.rear_clearance
            ly_eff = -ly                      # 진행축 반전 → 횡오프셋 부호 반전
        else:
            direction = 1.0
            heading_err = bearing
            clearance = self.front_clearance
            ly_eff = ly

        # 5) 방위 오차 크면 제자리 회전 (스키드스티어 활용)
        #    주변이 좁으면 회전 속도를 줄여 차체 모서리 충돌 위험 완화
        #    히스테리시스: heading_err 가 rotate_angle 경계에서 노이즈로 흔들려도
        #    rotate_exit_angle 까지 내려가야 해제 → 매 cycle 모드 토글(떨림) 방지
        if self._rotating:
            if abs(heading_err) < self.rotate_exit_angle:
                self._rotating = False
        elif abs(heading_err) > self.rotate_angle:
            self._rotating = True

        if self._rotating:
            # 제자리회전 각속도 상향(0.6→0.9배): 너무 느린 회전이 "기어가는" 체감 유발
            w = math.copysign(self.max_angular * 0.9, heading_err) * self._rot_scale()
            # EMA 를 경로추종과 공유 유지 → 모드 전환 시 각속도 급변(=진동) 제거
            w = self.ang_smooth * w + (1.0 - self.ang_smooth) * self._prev_ang
            self._prev_ang = w
            self.publish_cmd(0.0, w)
            return

        # 6) pure pursuit 곡률 → 속도 (전/후진 공통, 진행 방향 장애물 기준)
        dist = math.hypot(lx, ly)
        curvature = 2.0 * ly_eff / (dist * dist) if dist > 1e-6 else 0.0

        lin = self.linear_speed   # 크기(부호는 마지막에 direction 으로)
        # 진행 방향 장애물 정지 (히스테리시스): stop_dist 에서 멈추고,
        # stop_release_dist 이상 트여야 재출발 → clearance 노이즈로 인한 정지↔주행 토글 제거
        if self._stopped:
            if clearance > self.stop_release_dist:
                self._stopped = False
        elif clearance < self.stop_dist:
            self._stopped = True
        if self._stopped:
            self._prev_ang = 0.0      # 재출발 시 옛 각속도로 튀지 않게 EMA 리셋
            self.publish_cmd(0.0, 0.0)
            return
        if clearance < self.slow_down_dist:
            lin *= (clearance - self.stop_dist) / \
                   (self.slow_down_dist - self.stop_dist)
        # 방위 오차에 비례한 감속 (코너에서 안정). 분모를 heading_slow_angle 로 분리해
        # 주행 중 회전에서 선속도가 과하게 깎이지 않게 함 (하한 0.5).
        lin *= max(0.5, 1.0 - abs(heading_err) / self.heading_slow_angle)

        ang = max(-self.max_angular, min(self.max_angular, lin * curvature))
        ang *= self._rot_scale()    # 주변 좁으면 회전 성분도 감속
        # EMA 스무딩: 웨이포인트 전환 시 각속도 급변 → 진동 억제
        ang = self.ang_smooth * ang + (1.0 - self.ang_smooth) * self._prev_ang
        self._prev_ang = ang
        self.publish_cmd(lin * direction, ang)

    # ── 출력 ───────────────────────────────────────────────────
    def publish_cmd(self, lin: float, ang: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = lin
        msg.twist.angular.z = ang
        self.cmd_pub.publish(msg)

    def publish_stop(self):
        self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
