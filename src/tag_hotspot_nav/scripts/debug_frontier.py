"""frontier 미탐지 디버그: 맵 통계 + 로봇 셀 값 + frontier 탐지 단계별 확인."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import numpy as np

from tag_hotspot_nav.frontier_detection import detect_frontiers, is_new_frontier_cell
from tag_hotspot_nav.grid_utils import world_to_grid, get_cell_value, to_numpy


class Dbg(Node):
    def __init__(self):
        super().__init__('dbg_frontier')
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
        if n.map is not None and time.time() - t0 > 3:  # tf 채울 시간
            break
    m = n.map
    if m is None:
        print('맵 수신 실패'); return

    g = to_numpy(m)
    total = g.size
    print(f'맵 {m.info.width}x{m.info.height} res={m.info.resolution}')
    print(f'unknown={np.sum(g == -1)} ({100*np.sum(g==-1)/total:.0f}%)  '
          f'free={np.sum((g >= 0) & (g < 50))} ({100*np.sum((g>=0)&(g<50))/total:.0f}%)  '
          f'occ={np.sum(g >= 50)}')

    try:
        t = n.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        rx, ry = t.transform.translation.x, t.transform.translation.y
        print(f'로봇 pose=({rx:.2f},{ry:.2f})')
    except Exception as e:
        print(f'TF 실패: {e}'); return

    start = world_to_grid(m, rx, ry)
    print(f'로봇 grid={start}, 셀 값={get_cell_value(m, start)}')

    # 전체 맵에서 frontier 후보 셀 수 (BFS 도달성 무시)
    cnt = 0
    for gy in range(m.info.height):
        for gx in range(m.info.width):
            if is_new_frontier_cell(m, (gx, gy), {}):
                cnt += 1
    print(f'맵 전체 frontier 후보 셀: {cnt}')

    fs = detect_frontiers(m, start, 1)  # min_size=1
    print(f'BFS 도달 가능 frontier 군집(min_size=1): {len(fs)}, sizes={[f.size for f in fs]}')
    fs8 = detect_frontiers(m, start, 8)
    print(f'min_size=8 군집: {len(fs8)}')


main()
