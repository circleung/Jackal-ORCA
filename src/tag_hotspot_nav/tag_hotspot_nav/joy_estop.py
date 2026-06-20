import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class JoyEstop(Node):
    def __init__(self):
        super().__init__('joy_estop')
        self.stopped = False
        self._prev_x = 0

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub_platform = self.create_publisher(
            Bool, '/j100_0915/platform/safety_stop', latched)
        self._pub_pause = self.create_publisher(Bool, '/j100_0915/pause', 10)

        self.create_subscription(Joy, '/j100_0915/joy_teleop/joy', self._joy_cb, 10)
        self.get_logger().info('joy_estop 준비 — X버튼(인덱스0)으로 비상정지/재개')

    def _joy_cb(self, msg: Joy):
        if not msg.buttons:
            return
        x = msg.buttons[0]
        if x == 1 and self._prev_x == 0:          # rising edge
            self.stopped = not self.stopped
            m = Bool(data=self.stopped)
            self._pub_platform.publish(m)
            self._pub_pause.publish(m)
            if self.stopped:
                self.get_logger().warn('🛑 비상정지 — 전체 구동 차단 (X버튼으로 재개)')
            else:
                self.get_logger().info('▶ 비상정지 해제 — 주행 재개')
        self._prev_x = x


def main(args=None):
    rclpy.init(args=args)
    node = JoyEstop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
