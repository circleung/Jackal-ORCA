"""
footprint_marker.py — base_link 에 자칼 부피를 박스 Marker 로 발행하는 시각화 노드.

Foxglove 에서 URDF(robot_description)를 띄우려 하면 link 메시가 `package://...` 라
8766 브리지가 메시 파일을 못 가져와 link 마다 에러가 난다. 메시 로딩과 무관하게
로봇 부피를 보여주려고, base_link 프레임에 자칼 크기 CUBE Marker 하나를 발행한다.

Foxglove 3D 패널에서 이 토픽(/jackal_volume)을 추가하면, map→base_link TF 를 타고
로봇 위치에 박스가 따라다닌다 (TF 는 Foxglove 가 해석하므로 이 노드는 TF 불요).

치수 기본값(실측 기반):
  - 길이(x) 0.52m (중심→앞단 0.26m), 폭(y) 0.43m, 높이(z) 0.25m
  - base_link 지면높이 0.0635m 이라 박스 중심 z 오프셋 ≈ 0.06m (바닥~상판 0.25m 의 중앙)
모두 파라미터라 실측에 맞게 조정 가능.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from visualization_msgs.msg import Marker


class FootprintMarkerNode(Node):

    def __init__(self):
        super().__init__('footprint_marker')

        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('topic', '/jackal_volume')
        self.declare_parameter('size_x', 0.52)     # [m] 길이
        self.declare_parameter('size_y', 0.43)     # [m] 폭
        self.declare_parameter('size_z', 0.25)     # [m] 높이
        self.declare_parameter('z_offset', 0.06)   # [m] base_link 기준 박스 중심 z
        self.declare_parameter('rate', 2.0)        # [Hz] 재발행 주기
        # 색 (RGBA) — 반투명 노랑 (Clearpath 색감)
        self.declare_parameter('color', [1.0, 0.85, 0.0, 0.5])

        self.frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value
        self.sx = float(self.get_parameter('size_x').value)
        self.sy = float(self.get_parameter('size_y').value)
        self.sz = float(self.get_parameter('size_z').value)
        self.z_offset = float(self.get_parameter('z_offset').value)
        rate = float(self.get_parameter('rate').value)
        self.color = [float(c) for c in self.get_parameter('color').value]

        # transient-local: 늦게 붙는 Foxglove 가 마지막 Marker 를 바로 받게 (latched 유사)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Marker, topic, qos)

        self.create_timer(1.0 / rate, self.publish_marker)
        self.get_logger().info(
            f'footprint_marker 시작 — {topic} ({self.frame_id}, '
            f'{self.sx}x{self.sy}x{self.sz}m)')

    def publish_marker(self):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'jackal_volume'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = self.z_offset
        m.pose.orientation.w = 1.0
        m.scale.x = self.sx
        m.scale.y = self.sy
        m.scale.z = self.sz
        m.color.r, m.color.g, m.color.b, m.color.a = self.color
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FootprintMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
