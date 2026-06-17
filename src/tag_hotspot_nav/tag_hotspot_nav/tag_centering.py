"""
tag_centering.py — 매핑 중 태그를 잘 잡도록 잠깐 정렬하는 행동 노드.

젯슨 YOLO 가 /yolo/tag_candidate (base_link bearing) 로 "저기 태그 있다"를 발행해도
지금까지 아무도 안 받아서, 로봇이 태그를 더 잘 보려고 자세를 고치지 않았다. 이 노드는
front 카메라에 높은 신뢰 후보가 뜨면:
  1. 탐사 일시정지(/explore/command 'pause')
  2. bearing→0 이 되도록 제자리 회전(centering)
  3. dwell_time 동안 정지 유지 → tag_collector 가 또렷한 apriltag 관측 누적
  4. 탐사 재개(/explore/command 'resume') + cooldown (같은 태그 재트리거 방지)

cmd_vel 은 pause 중(pure_pursuit 정지)에만 발행하므로 충돌 없음. 수동 조이패드는
twist_mux 우선순위로 항상 우선. back 카메라 태그는 그대로 수동(passive) 수집.

입력:  /yolo/tag_candidate (custom_msgs/TagCandidate), /explore/command (String)
출력:  cmd_vel_topic (TwistStamped), /explore/command (String 'pause'/'resume')
"""
import math

import rclpy
from rclpy.node import Node

import rclpy.time
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from custom_msgs.msg import TagCandidate, TagPoseArray
from tf2_ros import Buffer, TransformListener

IDLE, CENTERING, DWELL, COOLDOWN = 'IDLE', 'CENTERING', 'DWELL', 'COOLDOWN'


class TagCenteringNode(Node):
    def __init__(self):
        super().__init__('tag_centering')

        self.declare_parameter('cmd_vel_topic', '/j100_0915/cmd_vel')
        self.declare_parameter('trigger_conf', 0.5)
        self.declare_parameter('center_tol', 0.12)        # [rad] 이 안이면 정렬 완료
        self.declare_parameter('angular_speed', 0.4)      # [rad/s] 정렬 회전속도
        self.declare_parameter('max_center_time', 4.0)    # [s] 정렬 타임아웃
        self.declare_parameter('dwell_time', 2.0)         # [s] 정렬 후 관측 정지
        self.declare_parameter('cooldown', 10.0)          # [s] 재트리거 방지
        self.declare_parameter('candidate_timeout', 1.5)  # [s] 후보 끊기면 태그 놓침
        self.declare_parameter('only_front', True)        # front 카메라 후보만 정렬
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('captured_count', 30)      # 이 이상 관측된 태그는 "포착 완료"
        self.declare_parameter('match_tol', 0.30)         # [rad] 후보 방향 vs 포착태그 방향 일치 허용

        self.cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.trig_conf = float(self.get_parameter('trigger_conf').value)
        self.tol       = float(self.get_parameter('center_tol').value)
        self.ang_spd   = float(self.get_parameter('angular_speed').value)
        self.max_ct    = float(self.get_parameter('max_center_time').value)
        self.dwell     = float(self.get_parameter('dwell_time').value)
        self.cool      = float(self.get_parameter('cooldown').value)
        self.cand_to   = float(self.get_parameter('candidate_timeout').value)
        self.only_front = bool(self.get_parameter('only_front').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.captured_count = int(self.get_parameter('captured_count').value)
        self.match_tol = float(self.get_parameter('match_tol').value)

        self.state = IDLE
        self._explore_active = True     # 외부 go/resume 시 True (탐사 중일 때만 동작)
        self._bearing = 0.0
        self._cand_t = None              # 마지막 후보 수신 시각
        self._t0 = 0.0                   # 상태 진입 시각
        self._captured = []              # 포착완료 태그 [(x,y)] (관측수 ≥ captured_count)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.ex_pub  = self.create_publisher(String, '/explore/command', 10)
        self.create_subscription(TagCandidate, '/yolo/tag_candidate', self._on_cand, 10)
        self.create_subscription(String, '/explore/command', self._on_cmd, 10)
        self.create_subscription(TagPoseArray, '/tags_in_map', self._on_tags, 10)

        self.create_timer(0.05, self._tick)   # 20 Hz
        self.get_logger().info(
            f'tag_centering up: front 후보 conf≥{self.trig_conf} → 정렬(±{self.tol}rad)'
            f' + {self.dwell}s 관측, cooldown {self.cool}s')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cand(self, msg: TagCandidate):
        if self.only_front and msg.source_camera != 'front':
            return
        if msg.confidence < self.trig_conf:
            return
        self._bearing = float(msg.bearing_rad)
        self._cand_t = self._now()

    def _on_cmd(self, msg: String):
        # 자기 cycle(non-IDLE) 중엔 자기 pause/resume 가 섞이므로 무시
        if self.state != IDLE:
            return
        c = msg.data.strip().lower()
        if c in ('go', 'resume', 'reset'):
            self._explore_active = True
        elif c == 'pause':
            self._explore_active = False

    def _pub_cmd(self, lin, ang):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.base_frame
        m.twist.linear.x = float(lin)
        m.twist.angular.z = float(ang)
        self.cmd_pub.publish(m)

    def _send(self, cmd):
        m = String(); m.data = cmd
        self.ex_pub.publish(m)

    def _cand_fresh(self):
        return self._cand_t is not None and (self._now() - self._cand_t) < self.cand_to

    def _on_tags(self, msg: TagPoseArray):
        # 충분히 관측된(포착완료) 태그의 map 위치만 보관
        self._captured = [(t.pose.pose.position.x, t.pose.pose.position.y)
                          for t in msg.tags if t.observation_count >= self.captured_count]

    def _already_captured(self):
        """현재 후보 방향에 '이미 포착완료한 태그'가 있으면 True (정렬 생략)."""
        if not self._captured:
            return False
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return False   # 포즈 모르면 판단 보류 → 정렬 진행
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        ryaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        for tx, ty in self._captured:
            tag_bearing = math.atan2(ty - ry, tx - rx) - ryaw
            diff = math.atan2(math.sin(tag_bearing - self._bearing),
                              math.cos(tag_bearing - self._bearing))
            if abs(diff) < self.match_tol:
                return True
        return False

    def _tick(self):
        now = self._now()
        if self.state == IDLE:
            if self._explore_active and self._cand_fresh():
                if self._already_captured():
                    # 같은 태그는 1번만 — 이미 포착한 태그면 정렬 생략, 짧은 쿨다운
                    self.state = COOLDOWN
                    self._t0 = now
                    self.get_logger().info('이미 포착한 태그 방향 — 정렬 생략',
                                           throttle_duration_sec=5.0)
                    return
                self.get_logger().info(
                    f'태그 후보 감지 (bearing={math.degrees(self._bearing):.0f}°) '
                    f'→ 탐사 정지·정렬')
                self._send('pause')
                self.state = CENTERING
                self._t0 = now

        elif self.state == CENTERING:
            if not self._cand_fresh():           # 태그 놓침 → 재개
                self.get_logger().info('후보 끊김 → 탐사 재개')
                self._pub_cmd(0.0, 0.0)
                self._send('resume')
                self.state = COOLDOWN; self._t0 = now
            elif abs(self._bearing) < self.tol or (now - self._t0) > self.max_ct:
                self._pub_cmd(0.0, 0.0)
                self.state = DWELL; self._t0 = now
            else:
                # bearing>0(좌)이면 +회전(좌), bearing<0(우)이면 -회전
                self._pub_cmd(0.0, math.copysign(self.ang_spd * min(1.0, abs(self._bearing) / 0.3), self._bearing))

        elif self.state == DWELL:
            self._pub_cmd(0.0, 0.0)              # 정지 유지 → 관측 누적
            if (now - self._t0) > self.dwell:
                self._send('resume')
                self.state = COOLDOWN; self._t0 = now

        elif self.state == COOLDOWN:
            if (now - self._t0) > self.cool:
                self.state = IDLE


def main(args=None):
    rclpy.init(args=args)
    node = TagCenteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
