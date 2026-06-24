"""
frontier_detection.py — BFS 기반 frontier 탐지.

KaiNakamura/slam_robot 의 frontier_detection.py 를 거의 그대로 가져옴.
변경점: slam_robot_interfaces 커스텀 msg 대신 dataclass 사용
(별도 인터페이스 패키지 없이 노드 내부에서 직접 사용).

frontier 정의: "unknown(-1) 셀이면서 free 셀과 4-연결로 인접한 셀"
→ 탐사가 덜 된 영역의 경계. 로봇이 거기로 가면 미지 영역이 밝혀진다.
"""

import math
from collections import deque
from dataclasses import dataclass, field

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

from tag_hotspot_nav.grid_utils import (
    FREE_THRESHOLD,
    get_cell_value,
    grid_to_world,
    get_neighbors_of_4,
    get_neighbors_of_8,
    is_cell_in_bounds,
)

MIN_FRONTIER_SIZE = 50  # 셀 수가 이보다 작은 frontier 는 노이즈로 무시


def _ray_free_steps(mapdata, gx, gy, dx, dy, max_cells):
    """(gx,gy)에서 (dx,dy) 방향으로 벽(occupied) 만나기 전까지 진행한 free/unknown
    셀 수. occupied(>=FREE_THRESHOLD) 또는 경계 밖에서 정지."""
    x, y = gx, gy
    steps = 0
    for _ in range(max_cells):
        x += dx
        y += dy
        if not is_cell_in_bounds(mapdata, (x, y)):
            break
        if get_cell_value(mapdata, (x, y)) >= FREE_THRESHOLD:   # 벽
            break
        steps += 1
    return steps


def passage_width_m(mapdata, gx, gy, max_range_m=2.5):
    """셀 (gx,gy)에서 4축(0/45/90/135°)으로 벽까지 양방향 거리를 재 최소 통로폭[m].
    free/unknown 은 통과하고 벽(occupied)에서만 정지 → 벽-사이 실제 폭을 측정한다.
    어느 축이든 벽이 max_range 안에 안 잡히면 그 축 폭은 크게 잡혀(=넓음) 필터 안 됨."""
    res = mapdata.info.resolution
    max_cells = max(1, int(max_range_m / res))
    min_w = float('inf')
    for dx, dy in ((1, 0), (0, 1), (1, 1), (1, -1)):
        pos = _ray_free_steps(mapdata, gx, gy, dx, dy, max_cells)
        neg = _ray_free_steps(mapdata, gx, gy, -dx, -dy, max_cells)
        steplen = math.hypot(dx, dy) * res
        width = (pos + neg + 1) * steplen   # +1: 중심 셀 포함
        if width < min_w:
            min_w = width
    return min_w


def _ray_wall_dist_m(mapdata, gx, gy, dx, dy, max_range_m):
    """(gx,gy)에서 (dx,dy) 방향(임의각 단위벡터)으로 벽까지의 거리[m].
    max_range_m 안에 벽이 없으면 max_range_m 그대로 반환(=그 방향은 뚫려있음)."""
    res = mapdata.info.resolution
    max_cells = max(1, int(max_range_m / res))
    x, y = gx + 0.5, gy + 0.5
    for step in range(max_cells):
        x += dx
        y += dy
        ci, cj = int(x), int(y)
        if not is_cell_in_bounds(mapdata, (ci, cj)):
            return max_range_m
        if get_cell_value(mapdata, (ci, cj)) >= FREE_THRESHOLD:   # 벽
            return step * res
    return max_range_m


def enclosure_ratio(mapdata, gx, gy, max_range_m=3.0, n_dirs=8):
    """frontier 주변 n_dirs 방향에서 max_range_m 안에 벽이 잡히는 비율(0~1).

    유리벽 안쪽 투영 frontier 구분용: 유리는 벽 틀(occupied)이 잡히지만 안쪽으로
    얇게 "뚫린 것처럼" 보이는 셀이 새어나가 frontier 가 생긴다. 진짜 출입구는
    최소 한 방향(문 쪽)은 끝까지 안 막혀있는데, 유리방은 그 "문처럼 보이는 틈"도
    실은 유리(벽 패턴)라 거의 모든 방향에서 벽이 잡힌다 → 비율이 높다(보통 0.75+).
    """
    hits = 0
    for i in range(n_dirs):
        ang = 2.0 * math.pi * i / n_dirs
        dx, dy = math.cos(ang), math.sin(ang)
        if _ray_wall_dist_m(mapdata, gx, gy, dx, dy, max_range_m) < max_range_m:
            hits += 1
    return hits / n_dirs


def passage_depth_m(mapdata, gx, gy, ux, uy, max_range_m=4.5, cone_deg=30.0):
    """(gx,gy)에서 바깥방향(ux,uy=로봇→frontier) 및 ±cone 으로 벽까지 free/unknown
    이 뻗는 최대 거리[m] = 그 공간이 얼마나 '깊은지'. 출입문(뒤에 방)=깊음,
    얕은 포켓=얕음. free/unknown 통과, 벽(occupied)/경계에서 정지."""
    res = mapdata.info.resolution
    max_cells = max(1, int(max_range_m / res))
    n = math.hypot(ux, uy) or 1.0
    ux, uy = ux / n, uy / n
    best = 0.0
    cone = math.radians(cone_deg)
    for ang in (0.0, cone, -cone):
        ca, sa = math.cos(ang), math.sin(ang)
        rx = ux * ca - uy * sa
        ry = ux * sa + uy * ca
        x, y = gx + 0.5, gy + 0.5
        steps = 0
        for _ in range(max_cells):
            x += rx
            y += ry
            ci, cj = int(x), int(y)
            if not is_cell_in_bounds(mapdata, (ci, cj)):
                break
            if get_cell_value(mapdata, (ci, cj)) >= FREE_THRESHOLD:   # 벽
                break
            steps += 1
        dist = steps * res
        if dist > best:
            best = dist
    return best


@dataclass
class Frontier:
    size: int
    centroid: Point
    cells: list = field(default_factory=list)  # world 좌표 Point 리스트


def is_new_frontier_cell(mapdata: OccupancyGrid, cell, is_frontier: dict) -> bool:
    """unknown 이고, 아직 frontier 로 표시 안 됐고, free 이웃이 하나 이상 있는가."""
    if not is_cell_in_bounds(mapdata, cell):
        return False

    if get_cell_value(mapdata, cell) != -1 or cell in is_frontier:
        return False

    for neighbor in get_neighbors_of_4(mapdata, cell, must_be_free=False):
        neighbor_value = get_cell_value(mapdata, neighbor)
        if 0 <= neighbor_value < FREE_THRESHOLD:
            return True

    return False


def build_new_frontier(mapdata: OccupancyGrid, initial_cell, is_frontier: dict) -> Frontier:
    """초기 frontier 셀에서 8-연결 BFS 로 인접 frontier 셀을 묶어 하나의 군집으로."""
    size = 1
    centroid_x = initial_cell[0]
    centroid_y = initial_cell[1]
    cells = [grid_to_world(mapdata, initial_cell)]

    queue = deque([initial_cell])
    while queue:
        current = queue.popleft()
        for neighbor in get_neighbors_of_8(mapdata, current, must_be_free=False):
            if is_new_frontier_cell(mapdata, neighbor, is_frontier):
                is_frontier[neighbor] = True
                size += 1
                centroid_x += neighbor[0]
                centroid_y += neighbor[1]
                cells.append(grid_to_world(mapdata, neighbor))
                queue.append(neighbor)

    centroid_x /= size
    centroid_y /= size
    centroid = grid_to_world(mapdata, (int(centroid_x), int(centroid_y)))

    return Frontier(size=size, centroid=centroid, cells=cells)


def detect_frontiers(mapdata: OccupancyGrid, start_pos,
                     min_size: int = MIN_FRONTIER_SIZE) -> list:
    """로봇 위치에서 free 영역을 4-연결 BFS 로 훑으며 frontier 군집을 수집.

    로봇이 도달 가능한 free 영역에 인접한 frontier 만 찾으므로,
    벽 너머 미지 영역은 자연히 제외된다.

    Returns:
        Frontier 리스트 (size >= min_size 만).
    """
    queue = deque([start_pos])
    visited = {start_pos: True}
    is_frontier = {}
    frontiers = []

    while queue:
        current = queue.popleft()
        # must_be_free=False: unknown 이웃도 열거해야 frontier 검사가 가능.
        # (True 면 free 셀만 나와 elif 분기에 절대 도달 못 하는 버그)
        for neighbor in get_neighbors_of_4(mapdata, current, must_be_free=False):
            neighbor_value = get_cell_value(mapdata, neighbor)
            if 0 <= neighbor_value < FREE_THRESHOLD and neighbor not in visited:
                visited[neighbor] = True
                queue.append(neighbor)
            elif is_new_frontier_cell(mapdata, neighbor, is_frontier):
                is_frontier[neighbor] = True
                new_frontier = build_new_frontier(mapdata, neighbor, is_frontier)
                if new_frontier.size >= min_size:
                    frontiers.append(new_frontier)

    return frontiers
