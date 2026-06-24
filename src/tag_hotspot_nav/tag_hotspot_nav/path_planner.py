"""
path_planner.py — occupancy grid 위 A* 전역 경로 계획 (Nav2 대체).

웹사이트(RBE3002 ROS1 버전) 설명을 따라 구현:
  - C-space: 장애물을 로봇 반경만큼 팽창 → 통행 가능 셀만 탐색
  - cost map 가중치: 벽에 가까울수록 비용 추가 → 복도 중앙 선호
  - start/goal 이 통행 불가 셀이면 가장 가까운 통행 가능 셀로 스냅
  - frontier 목표일 때 경로 끝 N 셀 절단 (미지 영역으로 과도 진입 방지)

순수 파이썬/numpy — ROS 노드 아님. frontier_explorer 가 호출한다.
"""

import heapq
import math

import numpy as np
from nav_msgs.msg import OccupancyGrid

from tag_hotspot_nav.grid_utils import (
    calc_cspace,
    calc_cost_map,
    grid_to_world,
    to_numpy,
    world_to_grid,
)

# 8방향 이동과 비용 (대각선 = √2)
_MOVES = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


def _octile(a, b):
    """8방향 격자에서 admissible 한 octile 거리 휴리스틱."""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)


def _rdp(pts, eps):
    """Ramer-Douglas-Peucker 경로 단순화: 직선에서 eps(m) 이내 점 제거.

    A* 경로의 미세 지그재그를 제거해 pure_pursuit 각속도 진동을 억제한다.
    """
    if len(pts) < 3:
        return pts
    x0, y0 = pts[0].x, pts[0].y
    x1, y1 = pts[-1].x, pts[-1].y
    dx, dy = x1 - x0, y1 - y0
    line_len = math.hypot(dx, dy)
    if line_len < 1e-9:
        dists = [math.hypot(p.x - x0, p.y - y0) for p in pts[1:-1]]
    else:
        dists = [abs(dx * (y0 - p.y) - (x0 - p.x) * dy) / line_len
                 for p in pts[1:-1]]
    max_d, max_i = max((d, i) for i, d in enumerate(dists))
    if max_d > eps:
        split = max_i + 1
        return _rdp(pts[:split + 1], eps)[:-1] + _rdp(pts[split:], eps)
    return [pts[0], pts[-1]]


class PathPlanner:
    """맵 1프레임에 대한 planner. 맵이 갱신되면 새로 만든다.

    C-space/costmap 계산은 생성 시 1회만 수행하므로,
    같은 맵으로 여러 frontier 후보를 평가할 때 재사용 가능.
    """

    def __init__(self, mapdata: OccupancyGrid,
                 robot_radius: float = 0.25,
                 cost_rings: int = 6,
                 ring_cost: float = 4.0):
        self.mapdata = mapdata
        self.grid = to_numpy(mapdata)
        self.padding = max(1, math.ceil(robot_radius / mapdata.info.resolution))
        self.walkable = calc_cspace(self.grid, self.padding)          # bool (h, w)
        self.cost_map = calc_cost_map(self.grid, self.padding,
                                      rings=cost_rings, ring_cost=ring_cost)

    # ── 스냅 ─────────────────────────────────────────────────────
    def nearest_walkable(self, cell, max_radius_cells: int = 40):
        """cell 에서 가장 가까운 통행 가능 셀 (BFS ring 탐색). 없으면 None."""
        h, w = self.walkable.shape
        gx, gy = cell
        if 0 <= gx < w and 0 <= gy < h and self.walkable[gy, gx]:
            return cell
        for r in range(1, max_radius_cells + 1):
            x0, x1 = max(0, gx - r), min(w - 1, gx + r)
            y0, y1 = max(0, gy - r), min(h - 1, gy + r)
            ring = self.walkable[y0:y1 + 1, x0:x1 + 1]
            ys, xs = np.nonzero(ring)
            if len(xs):
                # ring 내 통행 가능 셀 중 cell 에 가장 가까운 것
                cand = [(x0 + x, y0 + y) for x, y in zip(xs, ys)]
                return min(cand, key=lambda c: (c[0] - gx) ** 2 + (c[1] - gy) ** 2)
        return None

    # ── A* ──────────────────────────────────────────────────────
    def a_star(self, start, goal):
        """A* 탐색. Returns (cells 리스트, 총 이동 비용) 또는 (None, inf).

        goal 이 C-space 에서 단절돼 도달 불가면(부분 탐사 중 흔함),
        실패 대신 로봇 컴포넌트 내에서 goal 에 가장 가까운 지점까지의
        경로를 반환한다 → 로봇이 가장자리로 가면 라이다가 새 영역을
        밝혀 walkable 영역이 자라난다 (탐사의 닭-달걀 문제 해소).
        """
        h, w = self.walkable.shape
        walkable = self.walkable
        cost_map = self.cost_map

        open_heap = [(_octile(start, goal), 0.0, start)]
        g_score = {start: 0.0}
        came_from = {}
        closed = set()
        best_node, best_h = start, _octile(start, goal)   # goal 최근접 fallback

        while open_heap:
            _, g, current = heapq.heappop(open_heap)
            if current == goal:
                # 경로 복원
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path, g, True       # 실제 goal 도달
            if current in closed:
                continue
            closed.add(current)

            cur_h = _octile(current, goal)
            if cur_h < best_h:
                best_h, best_node = cur_h, current

            cx, cy = current
            for dx, dy, move_cost in _MOVES:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w and 0 <= ny < h) or not walkable[ny, nx]:
                    continue
                # 대각선 코너 커팅 금지: 양옆 셀이 모두 뚫려 있어야 통과
                if dx and dy and not (walkable[cy, nx] and walkable[ny, cx]):
                    continue
                neighbor = (nx, ny)
                # 이동 비용 + 벽 근접 패널티 (복도 중앙 선호)
                tentative = g + move_cost + cost_map[ny, nx]
                if tentative < g_score.get(neighbor, float('inf')):
                    g_score[neighbor] = tentative
                    came_from[neighbor] = current
                    heapq.heappush(
                        open_heap,
                        (tentative + _octile(neighbor, goal), tentative, neighbor))

        # goal 도달 불가 → 컴포넌트 내 goal 최근접 지점까지의 부분 경로
        # reached=False: 호출측이 "이 frontier 는 못 닿는다"고 알 수 있게 함
        # (부분경로는 진행용으로 발행하되, 도달해도 성공으로 치지 말 것)
        if best_node != start:
            path = [best_node]
            current = best_node
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path, g_score[best_node], False

        return None, float('inf'), False

    # ── 경로 직선화 (cost-aware string-pulling) ──────────────────
    def _line_clear(self, c0, c1, cost_budget):
        """Bresenham 직선상 모든 셀이 (1) C-space 통행 가능 AND (2) cost_map 값이
        cost_budget 이하인지. (2) 덕분에 직선화가 '원본 경로보다 벽에 더 붙는'
        지름길을 거부 → 복도 중앙 선호를 유지하면서도 넓은 곳은 직선이 된다."""
        x0, y0 = c0
        x1, y1 = c1
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        h, w = self.walkable.shape
        x, y = x0, y0
        while True:
            if not (0 <= x < w and 0 <= y < h and self.walkable[y, x]):
                return False
            if self.cost_map[y, x] > cost_budget:
                return False
            if x == x1 and y == y1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    # 하위호환(테스트용): 순수 통행가능 직선검사
    def _line_walkable(self, c0, c1):
        return self._line_clear(c0, c1, float('inf'))

    def _shortcut(self, cells):
        """cost-aware string-pulling: 가시선이 통하면서 '원본 부분경로가 지난
        최대 cost(=가장 벽에 가까웠던 정도)'를 넘지 않는 가장 먼 점으로 직선 연결.
        - 넓은 곳(cost≈0): 완전 직선화(노이즈 우회 loop 제거)
        - 복도/코너: 코너컷 지름길은 안쪽 벽 cost를 넘어 거부 → A* 중앙경로 유지"""
        if len(cells) <= 2:
            return cells
        n = len(cells)
        path_costs = [float(self.cost_map[c[1], c[0]]) for c in cells]
        eps = 1e-3
        out = [cells[0]]
        i = 0
        while i < n - 1:
            # cells[i..k] 의 prefix-max cost → 직선 j 후보가 넘으면 안 되는 한도
            run = path_costs[i]
            pm = [0.0] * n
            for k in range(i, n):
                if path_costs[k] > run:
                    run = path_costs[k]
                pm[k] = run
            j = n - 1
            while j > i + 1 and not self._line_clear(cells[i], cells[j], pm[j] + eps):
                j -= 1
            out.append(cells[j])
            i = j
        return out

    # ── 공개 API ─────────────────────────────────────────────────
    def plan(self, start_world, goal_world, truncate_end_cells: int = 0):
        """world 좌표 (x, y) 튜플 2개로 경로 계획.

        Args:
            truncate_end_cells: 경로 끝에서 잘라낼 셀 수.
                frontier 목표는 미지 영역 경계라 끝까지 가면 위험 →
                원본처럼 마지막 몇 셀을 잘라 알려진 영역에서 멈춘다.

        Returns:
            (world Point 리스트, 총비용) 또는 (None, inf)
        """
        start = self.nearest_walkable(world_to_grid(self.mapdata, *start_world))
        goal = self.nearest_walkable(world_to_grid(self.mapdata, *goal_world))
        if start is None or goal is None:
            return None, float('inf'), False

        cells, cost, reached = self.a_star(start, goal)
        if cells is None:
            return None, float('inf'), False

        if truncate_end_cells > 0 and len(cells) > truncate_end_cells + 2:
            cells = cells[:-truncate_end_cells]

        # string-pulling 직선화: 넓은 영역에서 격자 톱니·노이즈 우회로 생긴 꼬임을
        # 가시선 직선으로 편다(C-space 충돌검사 포함 → 안전). 이게 핵심 — RDP만으론
        # cost_map 이 벽/노이즈를 피해 휘게 만든 곡선을 못 편다.
        cells = self._shortcut(cells)
        world_pts = [grid_to_world(self.mapdata, c) for c in cells]
        # RDP: string-pulling 후 남은 미세 지그재그 정리 → pure_pursuit 각속도 진동 억제
        if len(world_pts) > 2:
            world_pts = _rdp(world_pts, 3.0 * self.mapdata.info.resolution)
        return world_pts, cost, reached
