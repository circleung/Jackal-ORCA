"""
cliff_stop.py — 젯슨 depth 계단 감지(/cliff_alert)를 받아 로봇을 정지시키는 브리지.

젯슨은 감지만(/cliff_alert Bool) 하고, 정지 판단·디바운스는 여기(mini-PC)서 한다.
연속 confirm_frames 프레임 true 면 /explore/command 'pause' 1회 발행 → frontier +
pure_pursuit 즉시 정지. 자동 재개는 하지 않는다(안전 — 사용자가 계단에서 벗어난 뒤
수동 resume). 경보가 rearm_frames 동안 false 면 재무장해 다음 계단에 다시 반응.

입력:  /cliff_alert (std_msgs/Bool, 젯슨 발행)
출력:  /explore/command (std_msgs/String 'pause')
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class CliffStopNode(Node):
    def __init__(self):
        super().__init__('cliff_stop')
        self.declare_parameter('confirm_frames', 3)   # 연속 true 이만큼이면 정지(플리커 방지)
        self.declare_parameter('rearm_frames', 8)     # 연속 false 이만큼이면 재무장
        self.confirm = int(self.get_parameter('confirm_frames').value)
        self.rearm = int(self.get_parameter('rearm_frames').value)

        self._true_run = 0
        self._false_run = 0
        self._latched = False

        self.create_subscription(Bool, '/cliff_alert', self._on_alert, 10)
        self.cmd_pub = self.create_publisher(String, '/explore/command', 10)
        self.get_logger().info(
            f'cliff_stop up: /cliff_alert {self.confirm}연속 true → pause '
            f'(자동 재개 없음, {self.rearm}연속 false 면 재무장)')

    def _on_alert(self, msg: Bool):
        if msg.data:
            self._true_run += 1
            self._false_run = 0
            if self._true_run >= self.confirm and not self._latched:
                self.cmd_pub.publish(String(data='pause'))
                self._latched = True
                self.get_logger().error(
                    '🛑 계단 감지(depth) → 탐사 정지. 안전 확인 후 수동 resume.')
        else:
            self._false_run += 1
            self._true_run = 0
            if self._latched and self._false_run >= self.rearm:
                self._latched = False
                self.get_logger().info('계단 경보 해제 — 재무장(다음 계단에 재반응)')


def main(args=None):
    rclpy.init(args=args)
    node = CliffStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
