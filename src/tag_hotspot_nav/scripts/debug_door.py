"""문 진입 실패 라이브 진단: /map_nav + robot_radius=0.30 (라이브와 동일).
   - C-space 연결성/컴포넌트
   - frontier별 스냅/컴포넌트내/A* 결과
   - frontier_explorer 와 동일한 narrow(통로폭/깊이) 필터 재현 → 어느 게 좁음제외인지
   - /map vs /map_nav 문 주변 비교 (map_cleaner 가 문을 막는지)
"""
import math, time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

from tag_hotspot_nav.frontier_detection import (
    detect_frontiers, passage_width_m, passage_depth_m)
from tag_hotspot_nav.grid_utils import world_to_grid, get_cell_value, to_numpy
from tag_hotspot_nav.path_planner import PathPlanner

ROBOT_RADIUS = 0.30      # 라이브 launch 값
MIN_PASS_W = 1.6         # 라이브 launch 값
MIN_PASS_D = 1.6
SKIP_DIST = 2.0          # 코드 기본값 (launch 미오버라이드)
MIN_FSIZE = 15


class Dbg(Node):
    def __init__(self):
        super().__init__('dbg_door')
        self.nav = None
        self.raw = None
        self.create_subscription(OccupancyGrid, '/map_nav', self._nav, 10)
        self.create_subscription(OccupancyGrid, '/map', self._raw, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _nav(self, m): self.nav = m
    def _raw(self, m): self.raw = m


def component_of(walkable, seed):
    h, w = walkable.shape
    comp = np.zeros_like(walkable)
    if not (0 <= seed[0] < w and 0 <= seed[1] < h and walkable[seed[1], seed[0]]):
        return comp
    q = deque([seed]); comp[seed[1], seed[0]] = True
    while q:
        x, y = q.popleft()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and walkable[ny, nx] and not comp[ny, nx]:
                    comp[ny, nx] = True; q.append((nx, ny))
    return comp


def dump_cspace(planner, cx, cy, label, half=14):
    w_ = planner.walkable
    h, w = w_.shape
    print(f'\n{label} C-space (■=walkable ·=막힘 X=중심):')
    for y in range(min(cy + half, h - 1), max(cy - half - 1, 0), -1):
        row = ''
        for x in range(max(cx - half, 0), min(cx + half + 1, w)):
            row += 'X' if (x, y) == (cx, cy) else ('■' if w_[y, x] else '·')
        print(row)


def dump_raw(grid, cx, cy, label, half=14):
    h, w = grid.shape
    print(f'\n{label} (·=free #=occ ?=unknown C=중심):')
    for y in range(min(cy + half, h - 1), max(cy - half - 1, 0), -1):
        row = ''
        for x in range(max(cx - half, 0), min(cx + half + 1, w)):
            if (x, y) == (cx, cy): row += 'C'; continue
            v = grid[y, x]
            row += '?' if v < 0 else ('#' if v >= 50 else '·')
        print(row)


def main():
    rclpy.init()
    n = Dbg()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 12:
        rclpy.spin_once(n, timeout_sec=0.3)
        if n.nav is not None and n.raw is not None and time.time() - t0 > 3:
            break
    m = n.nav
    if m is None:
        print('/map_nav 수신 실패'); return
    try:
        t = n.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        rx, ry = t.transform.translation.x, t.transform.translation.y
    except Exception as e:
        print(f'TF 실패: {e}'); return

    print(f'=== 로봇 pose=({rx:.2f},{ry:.2f})  맵 {m.info.width}x{m.info.height} '
          f'res={m.info.resolution} origin=({m.info.origin.position.x:.2f},'
          f'{m.info.origin.position.y:.2f}) ===')

    planner = PathPlanner(m, robot_radius=ROBOT_RADIUS)
    print(f'C-space walkable: {int(planner.walkable.sum())}/{planner.walkable.size} '
          f'(padding={planner.padding}셀={planner.padding*m.info.resolution:.2f}m)')
    start_raw = world_to_grid(m, rx, ry)
    start = planner.nearest_walkable(start_raw)
    print(f'로봇 raw={start_raw} 스냅={start}')
    comp = component_of(planner.walkable, start) if start else None
    print(f'로봇 컴포넌트 크기: {int(comp.sum()) if comp is not None else "N/A"}')

    fs = detect_frontiers(m, start_raw, MIN_FSIZE)
    print(f'\n=== frontier {len(fs)}개 (min_size={MIN_FSIZE}) — 라이브 필터 재현 ===')
    for f in fs:
        fx, fy = f.centroid.x, f.centroid.y
        g_raw = world_to_grid(m, fx, fy)
        g = planner.nearest_walkable(g_raw)
        in_comp = bool(comp[g[1], g[0]]) if (g and comp is not None) else None
        path, cost, reached = planner.plan((rx, ry), (fx, fy), truncate_end_cells=8)
        plan_s = (f'OK {len(path)}wp reached={reached}' if path else '실패')
        if path:
            endd = math.hypot(path[-1].x - rx, path[-1].y - ry)
            plan_s += f' 끝거리={endd:.2f}m'
        # narrow 필터 재현
        rdist = math.hypot(fx - rx, fy - ry)
        pw = passage_width_m(m, g_raw[0], g_raw[1], MIN_PASS_W + 1.0)
        dep = passage_depth_m(m, g_raw[0], g_raw[1], fx - rx, fy - ry, MIN_PASS_D + 2.5)
        if rdist <= SKIP_DIST:
            filt = 'KEEP(근처)'
        elif pw >= MIN_PASS_W:
            filt = 'KEEP(폭충분)'
        elif dep >= MIN_PASS_D:
            filt = 'KEEP(깊음=문)'
        else:
            filt = '★좁음제외★'
        print(f'  f({fx:+.2f},{fy:+.2f}) size={f.size} d={rdist:.2f}m '
              f'폭={pw:.2f} 깊이={dep:.2f} 컴포넌트내={in_comp} plan={plan_s} → {filt}')

    # 문 목표 (-1.48, 0.31) 집중
    DOOR = (-1.48, 0.31)
    dgx, dgy = world_to_grid(m, *DOOR)
    print(f'\n=== 문 목표 {DOOR} grid=({dgx},{dgy}) ===')
    dg = planner.nearest_walkable((dgx, dgy))
    print(f'  스냅={dg} 컴포넌트내={bool(comp[dg[1],dg[0]]) if (dg and comp is not None) else None}')
    dump_cspace(planner, dgx, dgy, f'문{DOOR} /map_nav robot_radius={ROBOT_RADIUS}')
    if n.raw is not None:
        graw = to_numpy(n.raw)
        rgx, rgy = world_to_grid(n.raw, *DOOR)
        dump_raw(graw, rgx, rgy, f'문{DOOR} /map(raw slam)')
        dump_raw(to_numpy(m), dgx, dgy, f'문{DOOR} /map_nav(cleaner)')


main()
