"""
frontier_detection.py — BFS 기반 frontier 탐지.

KaiNakamura/slam_robot 의 frontier_detection.py 를 거의 그대로 가져옴.
변경점: slam_robot_interfaces 커스텀 msg 대신 dataclass 사용
(별도 인터페이스 패키지 없이 노드 내부에서 직접 사용).

frontier 정의: "unknown(-1) 셀이면서 free 셀과 4-연결로 인접한 셀"
→ 탐사가 덜 된 영역의 경계. 로봇이 거기로 가면 미지 영역이 밝혀진다.
"""

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
