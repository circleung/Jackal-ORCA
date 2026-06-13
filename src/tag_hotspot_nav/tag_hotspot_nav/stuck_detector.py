"""
stuck_detector.py — 끼임(jam) 감지. 명령은 주는데 휠오돔이 안 움직이면 /stuck 발행.

장애물 앞에서 정상 정지(pure_pursuit)는 '명령 자체가 0' 이라 끼임이 아니다. 끼임은
케이블/턱에 걸려 "전진/회전 명령이 지속되는데도 로봇이 거의 안 움직이는" 상황.

입력:  cmd_topic(TwistStamped, 실제 주행명령), odom_topic(Odometry, 휠오돔)
출력:  /stuck (std_msgs/Bool) — 끼임 시 true (sound_player 가 pullup 재생)
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class StuckDetectorNode(Node):
    def __init__(self):
        super().__init__('stuck_detector')
        self.declare_parameter('cmd_topic', '/j100_0915/cmd_vel')
        self.declare_parameter('odom_topic', '/j100_0915/platform/odom')
        self.declare_parameter('lin_thresh', 0.05)   # m/s 이상이면 "전진 명령 중"
        self.declare_parameter('ang_thresh', 0.10)   # rad/s 이상이면 "회전 명령 중"
        self.declare_parameter('timeout', 5.0)       # s 명령 지속 + 무이동 판정
        self.declare_parameter('min_disp', 0.05)     # m 미만 이동이면 "안 움직임"
        self.declare_parameter('min_yaw', 0.08)      # rad 미만 회전이면 "안 돎"

        self.lin_th = float(self.get_parameter('lin_thresh').value)
        self.ang_th = float(self.get_parameter('ang_thresh').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.min_disp = float(self.get_parameter('min_disp').value)
        self.min_yaw = float(self.get_parameter('min_yaw').value)

        self._cmd = (0.0, 0.0)
        self._cmd_t = None
        self._cmd_since = None
        self._hist = deque()       # (t, x, y, yaw)
        self._stuck = False

        self.create_subscription(TwistStamped, self.get_parameter('cmd_topic').value,
                                 self._on_cmd, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self._on_odom, 10)
        self.pub = self.create_publisher(Bool, '/stuck', 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'stuck_detector up: 명령 지속 {self.timeout:.0f}s 무이동(<{self.min_disp}m)이면 /stuck')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        self._cmd = (float(msg.twist.linear.x), float(msg.twist.angular.z))
        self._cmd_t = self._now()

    def _on_odom(self, msg):
        t = self._now()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._hist.append((t, p.x, p.y, yaw))
        cutoff = t - (self.timeout + 2.0)
        while self._hist and self._hist[0][0] < cutoff:
            self._hist.popleft()

    def _tick(self):
        now = self._now()
        cmd_fresh = self._cmd_t is not None and (now - self._cmd_t) < 0.5
        commanding = cmd_fresh and (abs(self._cmd[0]) >= self.lin_th or
                                    abs(self._cmd[1]) >= self.ang_th)
        stuck = False
        if not commanding:
            self._cmd_since = None
        else:
            if self._cmd_since is None:
                self._cmd_since = now
            elif now - self._cmd_since >= self.timeout and self._hist:
                old = next((e for e in self._hist if now - e[0] <= self.timeout), None)
                cur = self._hist[-1]
                if old is not None and cur[0] - old[0] >= self.timeout * 0.7:
                    disp = math.hypot(cur[1] - old[1], cur[2] - old[2])
                    dyaw = abs(math.atan2(math.sin(cur[3] - old[3]),
                                          math.cos(cur[3] - old[3])))
                    stuck = disp < self.min_disp and dyaw < self.min_yaw

        if stuck and not self._stuck:
            self.get_logger().warn('🔧 끼임 감지 — 명령 지속에도 휠오돔 무이동')
        self._stuck = stuck
        self.pub.publish(Bool(data=stuck))


def main(args=None):
    rclpy.init(args=args)
    node = StuckDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
