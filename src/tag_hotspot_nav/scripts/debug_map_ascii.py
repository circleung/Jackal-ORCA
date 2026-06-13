"""맵 전체 ASCII 시각화 (3x3 다운샘플) + frontier/로봇 표시."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import numpy as np

from tag_hotspot_nav.frontier_detection import detect_frontiers
from tag_hotspot_nav.grid_utils import world_to_grid, to_numpy


class Dbg(Node):
    def __init__(self):
        super().__init__('dbg_map')
        self.map = None
        self.create_subscription(OccupancyGrid, '/map', self.cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def cb(self, msg):
        self.map = msg


def main():
    rclpy.init()
    n = Dbg()
    import time
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.5)
        if n.map is not None and time.time() - t0 > 3:
            break
    m = n.map
    g = to_numpy(m)
    t = n.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
    rx, ry = t.transform.translation.x, t.transform.translation.y
    rgx, rgy = world_to_grid(m, rx, ry)

    fs = detect_frontiers(m, (rgx, rgy), 8)
    fcells = set()
    for f in fs:
        for c in f.cells:
            fcells.add(world_to_grid(m, c.x, c.y))

    h, w = g.shape
    S = 3  # 다운샘플
    print(f'맵 {w}x{h} 로봇=({rx:.2f},{ry:.2f}) frontier {len(fs)}개 '
          f'sizes={sorted([f.size for f in fs], reverse=True)}')
    print('범례: #=벽 .=free (공백)=unknown F=frontier R=로봇 / 셀=15cm')
    for by in range(h - 1, -1, -S):
        row = ''
        for bx in range(0, w, S):
            blk = g[max(0, by - S + 1):by + 1, bx:bx + S]
            ch = ' '
            if np.any(blk >= 50):
                ch = '#'
            elif np.any((blk >= 0) & (blk < 50)):
                ch = '.'
            if any((bx <= fx < bx + S and by - S + 1 <= fy <= by)
                   for fx, fy in fcells):
                ch = 'F'
            if bx <= rgx < bx + S and by - S + 1 <= rgy <= by:
                ch = 'R'
            row += ch
        print(row)


main()
