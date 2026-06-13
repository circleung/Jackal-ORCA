#!/usr/bin/env python3
"""scan_selfcheck.py — /scan 의 방향별 최소거리를 출력해 '자기탐지(phantom)' 진단.

사용: 로봇 주변을 최대한 비운 뒤
    python3 ~/colcon_ws/scripts/scan_selfcheck.py
여러 프레임의 최소값을 모아 8방위로 보여줌. 장애물 없는 방향에 작은 값이 꾸준히
찍히면 = 차체 자기탐지. 그 거리 + 0.05 가 안전한 range_min.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

DIRS = ['전 0°', '전좌 45°', '좌 90°', '후좌 135°',
        '후 180°', '후우 225°', '우 270°', '전우 315°']


class Check(Node):
    def __init__(self):
        super().__init__('scan_selfcheck')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.cb, qos)
        self.acc = [math.inf] * 8     # 8방위 누적 최소
        self.gmin = math.inf
        self.gang = 0.0
        self.frames = 0
        self.create_timer(0.5, self.report)

    def cb(self, msg):
        self.frames += 1
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            a = msg.angle_min + i * msg.angle_increment
            s = int(((a + math.pi) / (2 * math.pi) * 8 + 4)) % 8   # 45° 버킷
            if r < self.acc[s]:
                self.acc[s] = r
            if r < self.gmin:
                self.gmin = r
                self.gang = math.degrees(a)

    def report(self):
        if self.frames < 5:
            return
        print(f"\n=== {self.frames} 프레임 누적 방향별 최소거리(m) ===")
        for d, v in zip(DIRS, self.acc):
            bar = '⚠ 의심' if v < 0.45 else ''
            print(f"  {d:10s}: {v:.2f} {bar}")
        print(f"  전체 최소: {self.gmin:.2f} m @ {self.gang:.0f}°")
        print("  → 장애물 없는 방향에 0.45 미만이면 그게 자기탐지 거리. "
              "range_min = 그값+0.05")
        rclpy.shutdown()


def main():
    rclpy.init()
    rclpy.spin(Check())


if __name__ == '__main__':
    main()
