"""경로 계획 실패 디버그: C-space 연결성 + frontier별 스냅/A* 결과."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import numpy as np
from collections import deque

from tag_hotspot_nav.frontier_detection import detect_frontiers
from tag_hotspot_nav.grid_utils import world_to_grid
from tag_hotspot_nav.path_planner import PathPlanner


class Dbg(Node):
    def __init__(self):
        super().__init__('dbg_planner')
        self.map = None
        self.create_subscription(OccupancyGrid, '/map', self.cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def cb(self, msg):
        self.map = msg


def component_of(walkable, seed):
    """seed 가 속한 8-연결 walkable 컴포넌트 마스크."""
    h, w = walkable.shape
    comp = np.zeros_like(walkable)
    if not walkable[seed[1], seed[0]]:
        return comp
    q = deque([seed])
    comp[seed[1], seed[0]] = True
    while q:
        x, y = q.popleft()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and walkable[ny, nx] and not comp[ny, nx]:
                    comp[ny, nx] = True
                    q.append((nx, ny))
    return comp


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
    t = n.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
    rx, ry = t.transform.translation.x, t.transform.translation.y

    planner = PathPlanner(m, robot_radius=0.25)
    print(f'C-space 통행 가능 셀: {planner.walkable.sum()} / 전체 {planner.walkable.size}')

    start_raw = world_to_grid(m, rx, ry)
    start = planner.nearest_walkable(start_raw)
    print(f'로봇 raw={start_raw} walkable={planner.walkable[start_raw[1], start_raw[0]]} → 스냅={start}')

    comp = component_of(planner.walkable, start)
    print(f'로봇 컴포넌트 크기: {comp.sum()}')

    fs = detect_frontiers(m, start_raw, 8)
    for f in fs:
        g_raw = world_to_grid(m, f.centroid.x, f.centroid.y)
        g = planner.nearest_walkable(g_raw)
        in_comp = comp[g[1], g[0]] if g else None
        path, cost = planner.plan((rx, ry), (f.centroid.x, f.centroid.y), truncate_end_cells=8)
        print(f'frontier size={f.size} centroid=({f.centroid.x:.2f},{f.centroid.y:.2f}) '
              f'goal스냅={g} 로봇컴포넌트내={in_comp} plan={"OK " + str(len(path)) + "wp" if path else "실패"}')

    # 로봇 주변 C-space 단면 출력 (21x21)
    gx, gy = start_raw
    h, w = planner.walkable.shape
    print('로봇 주변 C-space (■=walkable, ·=막힘, R=로봇):')
    for y in range(min(gy + 10, h - 1), max(gy - 11, 0), -1):
        row = ''
        for x in range(max(gx - 10, 0), min(gx + 11, w)):
            row += 'R' if (x, y) == (gx, gy) else ('■' if planner.walkable[y, x] else '·')
        print(row)


main()
