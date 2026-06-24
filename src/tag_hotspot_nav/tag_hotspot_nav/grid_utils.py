"""
grid_utils.py — OccupancyGrid 격자 연산 유틸리티.

KaiNakamura/slam_robot 의 frontier_utils.py 를 기반으로,
웹사이트(ROS1 RBE3002) 버전에 있던 C-space 팽창·costmap 생성을
numpy 로 추가한 것.

좌표 규약:
  grid (gx, gy)  : 셀 인덱스. data[gy * width + gx]
  world (x, y)   : map frame 미터 좌표
"""

import numpy as np
from scipy import ndimage
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

# 점유 판정 임계값: 0~49 free, 50~100 occupied, -1 unknown
FREE_THRESHOLD = 50

NEIGHBORS_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
NEIGHBORS_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1),
               (0, 1), (1, -1), (1, 0), (1, 1)]


def grid_to_index(mapdata: OccupancyGrid, p):
    return p[1] * mapdata.info.width + p[0]


def get_cell_value(mapdata: OccupancyGrid, p):
    return mapdata.data[grid_to_index(mapdata, p)]


def grid_to_world(mapdata: OccupancyGrid, p) -> Point:
    """셀 중심의 world 좌표."""
    x = (p[0] + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
    y = (p[1] + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y
    return Point(x=x, y=y, z=0.0)


def world_to_grid(mapdata: OccupancyGrid, wx: float, wy: float):
    gx = int((wx - mapdata.info.origin.position.x) / mapdata.info.resolution)
    gy = int((wy - mapdata.info.origin.position.y) / mapdata.info.resolution)
    return (gx, gy)


def is_cell_in_bounds(mapdata: OccupancyGrid, p) -> bool:
    return 0 <= p[0] < mapdata.info.width and 0 <= p[1] < mapdata.info.height


def is_cell_free(mapdata: OccupancyGrid, p) -> bool:
    """in-bounds 이고 알려진 free(< FREE_THRESHOLD, unknown 제외)인가."""
    if not is_cell_in_bounds(mapdata, p):
        return False
    v = get_cell_value(mapdata, p)
    return 0 <= v < FREE_THRESHOLD


def get_neighbors(mapdata: OccupancyGrid, p, directions, must_be_free=True):
    neighbors = []
    for dx, dy in directions:
        candidate = (p[0] + dx, p[1] + dy)
        if must_be_free:
            if is_cell_free(mapdata, candidate):
                neighbors.append(candidate)
        elif is_cell_in_bounds(mapdata, candidate):
            neighbors.append(candidate)
    return neighbors


def get_neighbors_of_4(mapdata, p, must_be_free=True):
    return get_neighbors(mapdata, p, NEIGHBORS_4, must_be_free)


def get_neighbors_of_8(mapdata, p, must_be_free=True):
    return get_neighbors(mapdata, p, NEIGHBORS_8, must_be_free)


# ──────────────────────────────────────────────────────────────────
# numpy 기반 C-space / costmap (원본 ROS1 path_planner.py 의
# calc_cspace / calc_cost_map 에 해당)
# ──────────────────────────────────────────────────────────────────

def to_numpy(mapdata: OccupancyGrid) -> np.ndarray:
    """OccupancyGrid.data → (height, width) int8 배열. [gy, gx] 인덱싱."""
    return np.asarray(mapdata.data, dtype=np.int8).reshape(
        mapdata.info.height, mapdata.info.width)


def _dilate(mask: np.ndarray, iterations: int) -> np.ndarray:
    """8-연결 binary dilation (scipy 없이 numpy roll 로 구현)."""
    out = mask.copy()
    for _ in range(iterations):
        d = out.copy()
        d[1:, :] |= out[:-1, :]
        d[:-1, :] |= out[1:, :]
        d[:, 1:] |= out[:, :-1]
        d[:, :-1] |= out[:, 1:]
        d[1:, 1:] |= out[:-1, :-1]
        d[1:, :-1] |= out[:-1, 1:]
        d[:-1, 1:] |= out[1:, :-1]
        d[:-1, :-1] |= out[1:, 1:]
        out = d
    return out


def despeckle(occupied: np.ndarray, min_blob_cells: int = 4) -> np.ndarray:
    """고립된 점 노이즈(유리 반사 등) 제거: 8-연결 컴포넌트 크기가
    min_blob_cells 미만이면 occupied 에서 제거.

    단순 erosion(모든 이웃이 occupied 여야 유지)은 두께 1셀짜리 진짜 벽까지
    지워버려 위험하다 — 그래서 "이웃 개수"가 아니라 "연결된 덩어리 전체 크기"로
    판단한다. 진짜 벽은 두께가 얇아도 길게 이어져 있어 컴포넌트 크기가 크므로
    안전하게 보존되고, 흩어진 노이즈 점(보통 1~3셀)만 제거된다.
    """
    if min_blob_cells <= 0:
        return occupied
    labeled, n = ndimage.label(occupied, structure=np.ones((3, 3), dtype=bool))
    if n == 0:
        return occupied
    sizes = ndimage.sum(occupied, labeled, index=np.arange(1, n + 1))
    small_labels = np.nonzero(sizes < min_blob_cells)[0] + 1
    if small_labels.size == 0:
        return occupied
    out = occupied.copy()
    out[np.isin(labeled, small_labels)] = False
    return out


def calc_cspace(grid: np.ndarray, padding_cells: int,
                despeckle_min_cells: int = 4) -> np.ndarray:
    """장애물을 로봇 반경만큼 팽창한 C-space 통행 가능 마스크.

    Args:
        grid: to_numpy() 결과 (h, w)
        padding_cells: 팽창 셀 수 (= robot_radius / resolution 올림)
        despeckle_min_cells: 이보다 작은 고립 점 노이즈는 팽창 전에 제거
            (유리 반사 등으로 생긴 점이 좁은 문/통로를 가짜로 막는 것 방지).
            0 이면 끔.

    Returns:
        walkable: bool (h, w). True = 통행 가능(알려진 free & 장애물에서 충분히 떨어짐)
    """
    occupied = grid >= FREE_THRESHOLD
    occupied = despeckle(occupied, despeckle_min_cells)
    inflated = _dilate(occupied, padding_cells)
    return (grid >= 0) & (grid < FREE_THRESHOLD) & ~inflated


def calc_cost_map(grid: np.ndarray, padding_cells: int,
                  rings: int = 6, ring_cost: float = 4.0,
                  despeckle_min_cells: int = 4) -> np.ndarray:
    """벽 근접도 비용 맵 — A* 가 복도 중앙을 선호하게 만든다.

    C-space 경계(팽창된 장애물)에서 바깥으로 ring 을 반복 팽창하며
    가까운 ring 일수록 높은 비용을 부여 (원본의 iterative dilation 방식).
    calc_cspace 와 동일하게 고립 점 노이즈를 먼저 제거해 일관성을 유지한다.

    Returns:
        cost: float32 (h, w). 0 = 벽에서 충분히 먼 곳, 클수록 벽에 가까움.
    """
    occupied = despeckle(grid >= FREE_THRESHOLD, despeckle_min_cells)
    occupied = _dilate(occupied, padding_cells)
    cost = np.zeros(grid.shape, dtype=np.float32)
    frontier = occupied
    for i in range(rings):
        expanded = _dilate(frontier, 1)
        new_ring = expanded & ~frontier
        cost[new_ring] = ring_cost * (rings - i) / rings
        frontier = expanded
    return cost
