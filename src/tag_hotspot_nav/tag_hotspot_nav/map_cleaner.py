"""
map_cleaner.py — 동적 장애물(지나가는 사람 등) 잔상 제거 레이어.

slam_toolbox 는 지나간 영역의 일시적 히트를 포즈그래프 맵에 구워버려, 사람이
떠나도 그 셀이 occupied 로 남는다. 여기선 현재 /scan 광선이 "통과해 그 너머까지
보이는" occupied 셀을 free 로 클리어한 /map_nav 를 발행한다 → planner/frontier 가
잔상 장애물에 막히지 않는다.

원리(폴라, 벡터화): 로봇 근처(clear_range) 의 각 occupied 셀에 대해, 그 방향의
현재 스캔 거리가 셀까지 거리보다 충분히 멀면(margin) = 빔이 그 셀을 통과 = 실제론
비어있음 → free. 실제 벽은 빔이 거기서 멈추므로 안 지워진다(정적 벽 보존).

입력:  /map (OccupancyGrid, slam), /scan (LaserScan, base_link), TF map→base_link
출력:  /map_nav (OccupancyGrid) — frontier/planner 가 /map 대신 구독
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class MapCleanerNode(Node):
    def __init__(self):
        super().__init__('map_cleaner')

        self.declare_parameter('clear_range', 6.0)    # [m] 로봇 주변 이 범위만 클리어
        self.declare_parameter('clear_margin', 0.25)  # [m] 빔이 셀보다 이만큼 더 멀어야 통과로 인정
        self.declare_parameter('occ_thresh', 50)      # 이 이상이면 occupied 로 간주
        self.declare_parameter('rate', 4.0)           # [Hz] 처리 주기
        self.declare_parameter('base_frame', 'base_link')
        # 동적 업데이트(매 틱 clear+scan마킹). off 면 /map_nav = slam맵 + 계단 keep-out 만(안정).
        # 매 틱 맵이 바뀌면 A* 가 흔들려 stop-start 가 잦아져서 기본 off (사용자 요청 2026-06-09)
        self.declare_parameter('dynamic_clean', False)

        self.clear_range  = float(self.get_parameter('clear_range').value)
        self.clear_margin = float(self.get_parameter('clear_margin').value)
        self.occ_thresh   = int(self.get_parameter('occ_thresh').value)
        self.base_frame   = self.get_parameter('base_frame').value
        self.dynamic_clean = bool(self.get_parameter('dynamic_clean').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._map = None
        self._scan = None
        # 영구 클리어: 런 내내 "빔이 통과해 비운" 셀을 월드셀 키(res 고정)로 누적.
        # 맵 origin/크기가 커져도 월드 기준이라 유효. → 지나간 영역 잔상도 유지 제거.
        self._cleared = set()       # {(kx, ky)} where kx=round(wx/res)
        self._res_key = 0.05        # 키 양자화 (맵 resolution 과 동일 가정)

        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scan_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(LaserScan, '/scan', self._on_scan, scan_qos)
        # 'reset' 시 영구 clear 셋 초기화 (이전 세션 맵핑 잔재 제거)
        self.create_subscription(String, '/explore/command', self._on_cmd, 10)
        self.pub = self.create_publisher(OccupancyGrid, '/map_nav', map_qos)

        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)
        self.get_logger().info(
            f'map_cleaner up: /map → /map_nav (clear_range={self.clear_range}m, '
            f'margin={self.clear_margin}m)')

    def _on_map(self, msg):
        self._map = msg

    def _on_scan(self, msg):
        self._scan = msg

    def _on_cmd(self, msg):
        if msg.data.strip().lower() == 'reset':
            nc = len(self._cleared)
            self._cleared.clear()
            self.get_logger().info(f'reset 수신 — clear({nc}) 셋 초기화')

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self._map.header.frame_id, self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def _tick(self):
        if self._map is None or self._scan is None:
            return
        m = self._map
        pose = self._robot_pose()
        if pose is None:
            self.pub.publish(m)   # 포즈 모르면 원본 통과
            return
        rx, ry, ryaw = pose
        res = m.info.resolution
        W, H = m.info.width, m.info.height
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        grid = np.array(m.data, dtype=np.int16).reshape(H, W)

        # 로봇 주변 윈도의 occupied 셀 — 빔이 통과하면 free 로 클리어 (occupied 없으면 건너뜀)
        occ = np.argwhere(grid >= self.occ_thresh)   # (gy,gx)
        if self.dynamic_clean and occ.size > 0:
            gy = occ[:, 0]; gx = occ[:, 1]
            cx = ox + (gx + 0.5) * res
            cy = oy + (gy + 0.5) * res
            dx = cx - rx; dy = cy - ry
            d = np.hypot(dx, dy)
            near = d < self.clear_range
            if near.any():
                gy = gy[near]; gx = gx[near]; d = d[near]
                bearing = np.arctan2(dy[near], dx[near]) - ryaw
                bearing = (bearing + math.pi) % (2 * math.pi) - math.pi   # [-pi,pi]
                s = self._scan
                idx = np.round((bearing - s.angle_min) / s.angle_increment).astype(int)
                ranges = np.asarray(s.ranges, dtype=np.float32)
                valid = (idx >= 0) & (idx < ranges.size)
                clear = np.zeros(gy.shape, dtype=bool)
                vi = np.where(valid)[0]
                r_at = ranges[idx[vi]]
                # 빔이 셀 너머까지 도달(유한 거리이고 셀보다 margin 이상 멀다) → 통과 → free
                passes = np.isfinite(r_at) & (r_at > d[vi] + self.clear_margin)
                clear[vi[passes]] = True
                if clear.any():
                    gyc = gy[clear]; gxc = gx[clear]
                    grid[gyc, gxc] = 0                 # 이번 스캔이 비운 셀
                    wxc = ox + (gxc + 0.5) * res
                    wyc = oy + (gyc + 0.5) * res
                    for kx, ky in zip(np.round(wxc / self._res_key).astype(int),
                                      np.round(wyc / self._res_key).astype(int)):
                        self._cleared.add((int(kx), int(ky)))

        # 과거에 비운 셀 전부 현재 격자에 다시 적용 (지나간 영역 잔상 유지 제거)
        if self.dynamic_clean and self._cleared:
            keys = np.array(tuple(self._cleared), dtype=np.float64)
            wx = keys[:, 0] * self._res_key
            wy = keys[:, 1] * self._res_key
            gxk = np.round((wx - ox) / res - 0.5).astype(int)
            gyk = np.round((wy - oy) / res - 0.5).astype(int)
            inb = (gxk >= 0) & (gxk < W) & (gyk >= 0) & (gyk < H)
            grid[gyk[inb], gxk[inb]] = 0

        # 현재 /scan 히트를 장애물로 마킹 — slam 이 놓치는 얇은/동적 장애물도 A* 가 피하게.
        # (장애물이 /map_nav 에 없어서 플래너가 같은 경로로 직진→정지 반복하던 문제 해결)
        sr = np.asarray(self._scan.ranges, dtype=np.float64)
        if self.dynamic_clean and sr.size:
            sa = self._scan.angle_min + np.arange(sr.size) * self._scan.angle_increment + ryaw
            lim = min(self._scan.range_max, self.clear_range)
            hit = np.isfinite(sr) & (sr >= self._scan.range_min) & (sr < lim)
            if hit.any():
                hx = rx + sr[hit] * np.cos(sa[hit])
                hy = ry + sr[hit] * np.sin(sa[hit])
                hgx = np.round((hx - ox) / res - 0.5).astype(int)
                hgy = np.round((hy - oy) / res - 0.5).astype(int)
                ib = (hgx >= 0) & (hgx < W) & (hgy >= 0) & (hgy < H)
                grid[hgy[ib], hgx[ib]] = 100

        out = OccupancyGrid()
        out.header = m.header
        out.info = m.info
        out.data = grid.astype(np.int8).reshape(-1).tolist()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MapCleanerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
