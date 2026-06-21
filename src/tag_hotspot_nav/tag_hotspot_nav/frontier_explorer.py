"""
frontier_explorer.py — frontier 탐지 + 선택 + A* 경로 발행 (탐사 두뇌).

원본 slam_robot 의 frontier_server + frontier_explorer + frontier_visualizer
세 노드를 하나로 합치고, Nav2 NavigateToPose 액션 대신
자체 PathPlanner(A*) 로 /plan 을 발행하도록 변경.

동작 주기 (1s 타이머):
  주행 중  → 목표 타임아웃 검사만
  대기 중  → frontier 탐지
             → 없음 ×30회 연속 = 탐사 완료 (/finish_exploration)
             → 있음: 1차 비용 상위 8개만 A* 로 정밀 평가
                     비용 = A*경로비용 × 진행방향편차가중 → 최저 선택
                     (A) DFS 식: 가까우면서 현재 진행방향에 가까운 frontier 우선
             → /plan 발행 → pure_pursuit 가 추종
             → /goal_reached 수신 시 다음 frontier
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from slam_toolbox.srv import Reset
from tf2_ros import Buffer, TransformListener

from tag_hotspot_nav.frontier_detection import MIN_FRONTIER_SIZE, detect_frontiers
from tag_hotspot_nav.grid_utils import world_to_grid
from tag_hotspot_nav.path_planner import PathPlanner


class FrontierExplorerNode(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # ── 파라미터 ─────────────────────────────────────────────
        self.declare_parameter('robot_radius', 0.25)        # [m] C-space 팽창
        self.declare_parameter('min_frontier_size', MIN_FRONTIER_SIZE)
        self.declare_parameter('min_distance', 0.5)         # [m] 너무 가까운 frontier 제외
        self.declare_parameter('top_n_astar', 8)            # A* 정밀 평가 후보 수 (원본 8)
        self.declare_parameter('truncate_end_cells', 8)     # 경로 끝 절단 (원본 8)
        self.declare_parameter('no_frontier_limit', 30)     # 종료 조건 (원본 30)
        self.declare_parameter('goal_timeout', 100.0)        # [s] 목표 타임아웃
        self.declare_parameter('no_progress_timeout', 30.0) # [s] 이만큼 거의 안 움직이면 재계획(막힘). 10→30: 느린주행/제자리회전을 막힘으로 오판해 목표를 자주 버리던 것 완화
        self.declare_parameter('no_progress_dist', 0.15)    # [m] 진전 판정 거리
        # (A) DFS 식 선택: 현재 진행방향과 어긋난 frontier 의 비용 가중 (0=순수 최근접,
        #     클수록 한 방향을 끝까지 파고듦). 정반대 방향이면 비용 ×(1+heading_weight).
        self.declare_parameter('heading_weight', 1.0)
        # goal 도달 후 AprilTag 스캔 회전 (front+back 카메라 합산 360° 커버)
        self.declare_parameter('scan_spin_duration', 7.0)   # [s] 7s × 0.5rad/s ≈ 200°
        self.declare_parameter('scan_spin_angular', 0.5)    # [rad/s]
        self.declare_parameter('cmd_vel_topic', '/j100_0915/cmd_vel')
        # 좁은 공간 탈출 후진
        self.declare_parameter('backup_duration', 2.5)      # [s]
        self.declare_parameter('backup_speed', -0.2)        # [m/s]
        # FIX-stuck-2: 후진 후 제자리회전 탈출 — 모서리는 직진 후진만으론 같은 데로 재진입.
        self.declare_parameter('escape_rotate_duration', 1.5)  # [s] 0이면 회전 탈출 off
        self.declare_parameter('escape_rotate_speed', 0.6)     # [rad/s]
        self.declare_parameter('blacklist_radius', 0.6)     # [m] 실패 frontier 회피 반경
        self.declare_parameter('blacklist_ttl', 60.0)       # [s] blacklist 유효 시간
        self.declare_parameter('blacklist_max_strikes', 3)  # 이 횟수 실패 시 영구 차단
        # 같은 자리(blacklist_radius 내) 목표에 "성공적으로" 도달한 횟수 제한.
        # 가구 등 고정 장애물에 가려진 포켓은 도달해도 못 밝혀져 매번 새 frontier로
        # 재등장 → 정상 도달은 blacklist 안 됨(166행) → 무한 재방문. 가구는 안 움직이므로
        # 같은 자리를 revisit_limit 회 넘게 갔으면 더 가도 소용없다고 보고 영구 차단.
        self.declare_parameter('revisit_limit', 2)

        self.robot_radius = self.get_parameter('robot_radius').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.min_distance = self.get_parameter('min_distance').value
        self.top_n_astar = self.get_parameter('top_n_astar').value
        self.truncate_end_cells = self.get_parameter('truncate_end_cells').value
        self.no_frontier_limit = self.get_parameter('no_frontier_limit').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.no_progress_timeout = self.get_parameter('no_progress_timeout').value
        self.no_progress_dist = self.get_parameter('no_progress_dist').value
        self.heading_weight = self.get_parameter('heading_weight').value
        self.scan_spin_duration = self.get_parameter('scan_spin_duration').value
        self.scan_spin_angular = self.get_parameter('scan_spin_angular').value
        self.backup_duration = self.get_parameter('backup_duration').value
        self.backup_speed = self.get_parameter('backup_speed').value
        self.escape_rotate_duration = self.get_parameter('escape_rotate_duration').value
        self.escape_rotate_speed = self.get_parameter('escape_rotate_speed').value
        self._np_pos = None      # 무진전 감지: 마지막 진전 위치
        self._np_t = None
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.blacklist_ttl = self.get_parameter('blacklist_ttl').value
        self.blacklist_max_strikes = self.get_parameter('blacklist_max_strikes').value
        self.revisit_limit = self.get_parameter('revisit_limit').value

        # ── 입출력 ──────────────────────────────────────────────
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_reached_sub = self.create_subscription(
            Bool, '/goal_reached', self.goal_reached_callback, 10)
        self.command_sub = self.create_subscription(
            String, '/explore/command', self.command_callback, 10)

        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        self.finish_pub = self.create_publisher(Bool, '/finish_exploration', 10)
        self.centroids_pub = self.create_publisher(MarkerArray, '/frontier_centroids', 10)
        self.cells_pub = self.create_publisher(GridCells, '/frontier_cells', 10)
        _cv_topic = self.get_parameter('cmd_vel_topic').value
        self._cmd_pub = self.create_publisher(TwistStamped, _cv_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 'go' 때 이전 맵을 지우기 위한 slam_toolbox 리셋 클라이언트
        self.slam_reset_client = self.create_client(Reset, '/slam_toolbox/reset')

        # ── 상태 ────────────────────────────────────────────────
        self.mapdata = None
        self.is_navigating = False
        self.goal_start_time = None
        self.no_frontiers_count = 0
        self.all_failed_count = 0    # 연속 "전 후보 진행 불가" 사이클 수
        # 실패 frontier 기록 {(x, y): [마지막 실패 시각, 누적 실패 횟수]}
        # TTL 안에는 회피, TTL 지나면 재시도 자격 (도달 직후 일시 정체로
        # 영구 차단되는 문제 방지). max_strikes 누적 시에만 영구 차단.
        self.blacklist = {}
        self.visit_counts = {}   # {(x, y): 성공 도달 횟수} — revisit_limit 판정용
        self.finished = False
        self.paused = True   # 'go' 명령 전까지 대기
        self.final_sweep_done = False   # 종료 전 blacklist 리셋 재확인 1회
        self.current_goal = None        # 현재 추종 중인 frontier 중심 (막힘 시 blacklist 대상)
        self._goal_partial = False      # 현재 목표가 부분경로(도달 불가)인지
        self._np_retry_goal = None      # 무진전 1회 재시도한 목표 (재발 시에만 포기)
        # 스캔 회전 / 후진 상태
        self._scanning = False
        self._scan_end_time = None
        self._backing_up = False
        self._backup_end_time = None
        self._rotate_escape = False   # FIX-stuck-2: 후진 후 제자리회전 탈출 상태
        self._rotate_end_time = None

        self.timer = self.create_timer(1.0, self.explore_step)
        self.create_timer(0.1, self._motion_cb)
        self.get_logger().info("frontier_explorer 대기 중 — 터미널에서 'go' 입력으로 시작")

    # ── 콜백 ───────────────────────────────────────────────────
    def map_callback(self, msg: OccupancyGrid):
        self.mapdata = msg

    def _motion_cb(self):
        """스캔 회전 / 후진 탈출 cmd_vel 발행 (0.1s 주기)."""
        now = self.get_clock().now()
        twist = TwistStamped()
        twist.header.stamp = now.to_msg()
        twist.header.frame_id = 'base_link'
        if self._scanning:
            if self._scan_end_time and now < self._scan_end_time:
                twist.twist.angular.z = self.scan_spin_angular
            else:
                self._scanning = False
                self.get_logger().info('스캔 회전 완료 → 다음 frontier 탐색')
            self._cmd_pub.publish(twist)
        elif self._backing_up:
            if self._backup_end_time and now < self._backup_end_time:
                twist.twist.linear.x = self.backup_speed
            else:
                self._backing_up = False
                # FIX-stuck-2: 후진 후 제자리회전으로 헤딩 전환(모서리 재진입 방지)
                if self.escape_rotate_duration > 0:
                    self._rotate_escape = True
                    self._rotate_end_time = (now + Duration(
                        seconds=self.escape_rotate_duration))
                    self.get_logger().info('후진 완료 → 제자리회전 탈출')
                else:
                    self.get_logger().info('후진 탈출 완료 → 재계획')
            self._cmd_pub.publish(twist)
        elif self._rotate_escape:
            if self._rotate_end_time and now < self._rotate_end_time:
                twist.twist.angular.z = self.escape_rotate_speed
            else:
                self._rotate_escape = False
                self.get_logger().info('제자리회전 탈출 완료 → 재계획')
            self._cmd_pub.publish(twist)

    def _record_visit(self, cx: float, cy: float):
        """frontier 목표 성공 도달 기록. 같은 자리(blacklist_radius 내) revisit_limit
        회 넘게 도달했는데도 frontier 가 계속 재등장하면 = 가구 등 고정 장애물로
        영영 못 밝히는 자리 → 더 가봐도 소용없으니 영구 차단(blacklist max_strikes)."""
        for (x, y), cnt in list(self.visit_counts.items()):
            if math.hypot(cx - x, cy - y) < self.blacklist_radius:
                cnt += 1
                self.visit_counts[(x, y)] = cnt
                if cnt >= self.revisit_limit:
                    self.blacklist[(x, y)] = [self.now_sec(), self.blacklist_max_strikes]
                    self.get_logger().warn(
                        f'같은 자리 ({x:.2f},{y:.2f}) {cnt}회 도달해도 새 영역 안 밝혀짐 '
                        '→ 고정 장애물(가구 등)로 판단, 영구 차단')
                return
        self.visit_counts[(cx, cy)] = 1

    def goal_reached_callback(self, msg: Bool):
        if msg.data and self.is_navigating:
            self.is_navigating = False
            self._np_retry_goal = None
            if self.current_goal is not None:
                self._record_visit(*self.current_goal)
                # 부분경로(도달 불가) 목표는 끝점 도달=영역 미밝힘 → blacklist 해서
                # 다음 사이클에 같은 frontier 재선택(반복 루트)을 끊는다.
                if self._goal_partial:
                    self.add_to_blacklist(*self.current_goal)
                    self.get_logger().warn(
                        f'도달불가 frontier ({self.current_goal[0]:.2f},'
                        f'{self.current_goal[1]:.2f}) 부분경로 끝 도달 → blacklist')
            if self.scan_spin_duration > 0:
                self._scanning = True
                self._scan_end_time = (self.get_clock().now()
                                       + Duration(seconds=self.scan_spin_duration))
                self.get_logger().info(
                    f'목표 도달 → {self.scan_spin_duration:.0f}s 스캔 회전 (AprilTag 탐색)')
            else:
                self.get_logger().info('목표 도달 → 다음 frontier 탐색')
            self.current_goal = None   # 정상 도달은 blacklist 안 함

    def command_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ('go', 'resume'):
            # 시작/재개 — 맵·상태는 그대로 두고 탐사만 (재)시작. (초기화는 'reset')
            if not self.paused and not self.finished:
                return   # 이미 탐사 중 (1초 반복 발행 중복 무시)
            self.paused = False
            self.finished = False
            self.get_logger().info("명령 'go' → 탐사 시작/재개")
        elif cmd == 'reset':
            # 매핑 처음부터: 이전 맵 폐기(slam 리셋) + 상태 전부 초기화 후 시작
            self.plan_pub.publish(Path())   # 혹시 남은 경로 정지
            if self.slam_reset_client.service_is_ready():
                self.slam_reset_client.call_async(Reset.Request())
                self.mapdata = None         # 새 /map 수신까지 대기
                self.get_logger().info('이전 맵 리셋 요청 → 새 맵으로 시작')
            else:
                self.get_logger().warn('slam_toolbox reset 서비스 없음 — 기존 맵 유지')
            self.finished = False
            self.paused = False
            self.is_navigating = False
            self.no_frontiers_count = 0
            self.all_failed_count = 0
            self.blacklist = {}
            self.visit_counts = {}
            self.final_sweep_done = False
            self._np_retry_goal = None
            self.get_logger().info("명령 'reset' → 매핑 처음부터 다시")
        elif cmd == 'pause':
            if self.paused:
                return
            self.paused = True
            self.is_navigating = False
            self.plan_pub.publish(Path())   # 빈 경로 → pure_pursuit 정지
            self.get_logger().info("명령 'pause' → 일시정지")

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            tr = t.transform.translation
            return tr.x, tr.y
        except Exception:
            return None

    def _robot_yaw(self):
        """현재 로봇 yaw(map 기준). 진행방향 연속성 가중에 사용. 실패 시 None."""
        try:
            q = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()).transform.rotation
            return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except Exception:
            return None

    def _heading_factor(self, fx, fy, rx, ry, ryaw):
        """frontier 방향이 현재 진행방향과 어긋난 정도에 따른 비용 배율.
        정면=1.0, 정반대=1+heading_weight. ryaw 없으면 1.0(순수 거리)."""
        if ryaw is None:
            return 1.0
        bearing = math.atan2(fy - ry, fx - rx)
        head_diff = abs(math.atan2(math.sin(bearing - ryaw), math.cos(bearing - ryaw)))
        return 1.0 + self.heading_weight * head_diff / math.pi

    # ── 메인 루프 ──────────────────────────────────────────────
    def explore_step(self):
        if self._scanning or self._backing_up or self._rotate_escape:
            return
        if self.paused or self.finished or self.mapdata is None:
            return

        # 주행 중이면 타임아웃 + 무진전(막힘) 검사
        if self.is_navigating:
            now = self.get_clock().now()
            elapsed = (now - self.goal_start_time).nanoseconds * 1e-9
            if elapsed > self.goal_timeout:
                if self.current_goal is not None:
                    self.add_to_blacklist(*self.current_goal)
                self.get_logger().warn(
                    f'목표 타임아웃({self.goal_timeout:.0f}s) → 목표 blacklist 후 재계획')
                self.is_navigating = False
                self._np_pos = None
                self._np_retry_goal = None
                return
            # 무진전 재계획: 장애물 막힘 등으로 10초간 거의 안 움직이면 즉시 재계획
            pose = self.get_robot_pose()
            tnow = now.nanoseconds * 1e-9
            if pose is not None:
                if (self._np_pos is None or
                        math.hypot(pose[0] - self._np_pos[0], pose[1] - self._np_pos[1])
                        > self.no_progress_dist):
                    self._np_pos = pose
                    self._np_t = tnow
                elif tnow - self._np_t > self.no_progress_timeout:
                    # 경로 커밋: 첫 무진전이면 같은 목표를 버리지 않고 후진으로 자세만
                    # 회복한 뒤 동일 목표로 재계획(경로 최대한 유지). 같은 목표에서
                    # 또 무진전하면 그때 blacklist 해서 다른 경로로 전환한다.
                    g = self.current_goal
                    if g is not None:
                        retried = (self._np_retry_goal is not None and
                                   math.hypot(g[0] - self._np_retry_goal[0],
                                              g[1] - self._np_retry_goal[1])
                                   < self.blacklist_radius)
                        if retried:
                            self.add_to_blacklist(*g)
                            self._np_retry_goal = None
                            self.get_logger().warn(
                                f'무진전 재발 → 목표 ({g[0]:.2f},{g[1]:.2f}) '
                                'blacklist 후 다른 경로로 전환')
                        else:
                            self._np_retry_goal = g
                            self.get_logger().warn(
                                f'무진전({self.no_progress_timeout:.0f}s) → 후진 후 '
                                f'같은 목표 ({g[0]:.2f},{g[1]:.2f}) 재계획(경로 유지 시도)')
                    else:
                        self.get_logger().warn(
                            f'무진전({self.no_progress_timeout:.0f}s, 막힘 추정) → 후진 후 재계획')
                    # 좁은 공간 탈출: 후진
                    if self.backup_duration > 0:
                        # FIX-stuck-1(A): 후진 핸드오프 — 빈 /plan 으로 pure_pursuit 를
                        # 정지시켜 cmd_vel 을 양보받는다. 안 그러면 pure_pursuit 가 옛 경로로
                        # 0(정지)을 계속 발행해 후진(-0.2)과 경합 → 실제로 안 물러남.
                        self.plan_pub.publish(Path())
                        self._backing_up = True
                        self._backup_end_time = (self.get_clock().now()
                                                 + Duration(seconds=self.backup_duration))
                    self.is_navigating = False
                    self._np_pos = None
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn('TF map→base_link 대기 중', throttle_duration_sec=10.0)
            return

        # 1) frontier 탐지
        mapdata = self.mapdata
        start_grid = world_to_grid(mapdata, *robot_pose)
        frontiers = detect_frontiers(mapdata, start_grid, self.min_frontier_size)
        frontiers = [f for f in frontiers if not self.is_blacklisted(f.centroid)]
        self.publish_visualization(frontiers, mapdata)

        # 2) 종료 판정
        if not frontiers:
            self.no_frontiers_count += 1
            self.get_logger().info(
                f'frontier 없음 ({self.no_frontiers_count}/{self.no_frontier_limit})')
            if self.no_frontiers_count >= self.no_frontier_limit:
                if self.try_final_sweep():
                    return
                self.get_logger().info('탐사 완료!')
                self.finished = True
                self.plan_pub.publish(Path())          # 빈 경로 → 정지
                self.finish_pub.publish(Bool(data=True))
            return
        self.no_frontiers_count = 0

        # 3) 1차 선별 (싸게) → 상위 N 개만 A* 평가 (비싸게).
        #    (A) DFS 식: 전역 '큰 frontier' 선호(d/size)를 버리고, 가까우면서 현재
        #    진행방향에 가까운 frontier 를 우선 → 한 방향을 끝까지 파고듦(핑퐁 억제).
        ryaw = self._robot_yaw()
        def euclid_cost(f):
            d = math.hypot(f.centroid.x - robot_pose[0], f.centroid.y - robot_pose[1])
            if d < self.min_distance:
                return float('inf')
            return d * self._heading_factor(
                f.centroid.x, f.centroid.y, robot_pose[0], robot_pose[1], ryaw)

        ranked = [f for f in sorted(frontiers, key=euclid_cost)
                  if euclid_cost(f) != float('inf')]
        candidates = ranked[:self.top_n_astar]
        if not candidates:
            return

        planner = PathPlanner(mapdata, robot_radius=self.robot_radius)

        # 로봇 위치가 C-space 에서 스냅조차 안 되면 로봇 쪽 문제(맵 갱신 중 등)
        # → frontier 탓이 아니므로 blacklist 없이 이번 사이클만 건너뜀
        if planner.nearest_walkable(start_grid) is None:
            self.get_logger().warn('로봇 위치 C-space 스냅 실패 → 사이클 건너뜀')
            return

        best_path, best_cost = None, float('inf')
        best_frontier = None
        best_reached = False   # 선택된 목표가 실제 goal 도달 가능 경로인지(부분경로 아님)
        cycle_blacklist = []   # 이번 사이클에서 실패한 후보 (확정 전 임시 보관)
        for f in candidates:
            path, cost, reached = planner.plan(
                robot_pose, (f.centroid.x, f.centroid.y),
                truncate_end_cells=self.truncate_end_cells)
            if path is None:
                cycle_blacklist.append((f.centroid.x, f.centroid.y))
                continue
            # 절단 후 경로 끝이 사실상 제자리면(라이다 음영 등) 이동해도
            # 못 밝히는 frontier → blacklist (제자리 도달 무한루프 방지)
            if math.hypot(path[-1].x - robot_pose[0],
                          path[-1].y - robot_pose[1]) < 0.3:
                cycle_blacklist.append((f.centroid.x, f.centroid.y))
                continue
            # (A) DFS 식: A* 경로비용 × 진행방향 편차 가중 (size 미사용 = 최근접·동방향 우선)
            cost *= self._heading_factor(
                f.centroid.x, f.centroid.y, robot_pose[0], robot_pose[1], ryaw)
            if cost < best_cost:
                best_path, best_cost, best_frontier, best_reached = path, cost, f, reached

        # 상위 N개 전멸 시 나머지 frontier 도 순서대로 평가 (첫 성공 채택).
        # 좁은 C-space 포켓에선 가까운 대형 frontier 는 진행 불가여도
        # 멀리 반대편 frontier 로는 빠져나갈 수 있다 (실사례: 서쪽 포켓에
        # 갇혔을 때 동쪽 frontier 만 0.85m 진행 가능했는데 상위 8개에 못 듦)
        if best_path is None:
            for f in ranked[self.top_n_astar:]:
                path, cost, reached = planner.plan(
                    robot_pose, (f.centroid.x, f.centroid.y),
                    truncate_end_cells=self.truncate_end_cells)
                if path is None:
                    continue
                if math.hypot(path[-1].x - robot_pose[0],
                              path[-1].y - robot_pose[1]) < 0.3:
                    continue
                best_path, best_frontier, best_reached = path, f, reached
                break

        if best_path is None:
            # 전 후보 동시 실패 = 로봇 쪽 일시 문제일 가능성이 높음
            # (실제 사례: 도착 직후 1사이클 실패로 후보 8개 전부 영구 blacklist
            #  → 탐사 조기 종료). 이번 사이클 blacklist 는 버린다.
            # 단, 이 상태가 지속되면 = 어디로도 진행 불가 → 탐사 종료
            self.all_failed_count += 1
            self.get_logger().warn(
                f'모든 후보({len(candidates)}) 진행 불가 '
                f'({self.all_failed_count}/{self.no_frontier_limit})',
                throttle_duration_sec=5.0)
            if self.all_failed_count >= self.no_frontier_limit:
                if self.try_final_sweep():
                    return
                self.get_logger().info('진행 가능한 frontier 없음 → 탐사 완료!')
                self.finished = True
                self.plan_pub.publish(Path())
                self.finish_pub.publish(Bool(data=True))
            return
        self.all_failed_count = 0
        self.final_sweep_done = False   # 진행 재개 → 다음 정체 때 스윕 재허용
        # 성공한 사이클의 실패 후보만 blacklist 에 기록 (TTL + strike 방식)
        for cx, cy in cycle_blacklist:
            self.add_to_blacklist(cx, cy)

        # 4) 경로 발행
        self.publish_plan(best_path)
        self.is_navigating = True
        # 막힘(goal_timeout·무진전) 시 blacklist 대상이 될 현재 목표 기록.
        # (이게 None 이면 blacklist 분기가 죽어 같은 frontier 를 무한 재선택함)
        self.current_goal = (best_frontier.centroid.x, best_frontier.centroid.y)
        # 부분경로(도달 불가) 목표는 끝점에 "도달"해도 그 영역이 안 밝혀지므로,
        # 도달 시 성공으로 치지 말고 blacklist 해야 같은 frontier 무한 재선택을 끊는다.
        self._goal_partial = not best_reached
        # 재시도 추적: 다른 목표로 넘어갔으면 무진전 재시도 상태 초기화
        if (self._np_retry_goal is not None and
                math.hypot(self.current_goal[0] - self._np_retry_goal[0],
                           self.current_goal[1] - self._np_retry_goal[1])
                >= self.blacklist_radius):
            self._np_retry_goal = None
        self.goal_start_time = self.get_clock().now()
        self.get_logger().info(
            f'frontier 목표 ({best_frontier.centroid.x:.2f}, '
            f'{best_frontier.centroid.y:.2f}) size={best_frontier.size}, '
            f'{len(best_path)} waypoints')

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def try_final_sweep(self) -> bool:
        """종료 선언 전 1회: blacklist 를 비우고 전체 frontier 재평가.

        strike 누적으로 멀쩡한 frontier 까지 차단된 채 조기 종료하는
        문제 방지 (실사례: 진행 가능 frontier 4개가 전부 blacklist 상태로
        '후보 1개 진행 불가' 종료). 수행했으면 True (종료 보류).
        """
        if self.final_sweep_done:
            return False
        self.final_sweep_done = True
        self.blacklist = {}
        self.no_frontiers_count = 0
        self.all_failed_count = 0
        self.get_logger().info('종료 전 최종 재확인 — blacklist 초기화 후 전체 재평가')
        return True

    def is_blacklisted(self, centroid) -> bool:
        now = self.now_sec()
        for (x, y), (stamp, strikes) in self.blacklist.items():
            if math.hypot(centroid.x - x, centroid.y - y) < self.blacklist_radius:
                if strikes >= self.blacklist_max_strikes:   # 영구 차단
                    return True
                if now - stamp < self.blacklist_ttl:        # 아직 TTL 내
                    return True
        return False

    def add_to_blacklist(self, cx: float, cy: float):
        """기존 항목 반경 내면 strike 누적, 아니면 새 항목."""
        for (x, y), rec in self.blacklist.items():
            if math.hypot(cx - x, cy - y) < self.blacklist_radius:
                rec[0] = self.now_sec()
                rec[1] += 1
                return
        self.blacklist[(cx, cy)] = [self.now_sec(), 1]

    # ── 발행 ───────────────────────────────────────────────────
    def publish_plan(self, points):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for p in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position = p
            ps.pose.orientation = Quaternion(w=1.0)
            path.poses.append(ps)
        self.plan_pub.publish(path)

    def publish_visualization(self, frontiers, mapdata):
        """frontier 중심(구 마커) + 셀(GridCells) RViz 시각화 (원본 visualizer 통합)."""
        marker_array = MarkerArray()
        for i, f in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = f.centroid
            m.pose.orientation = Quaternion(w=1.0)
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.b = 1.0
            m.color.a = 1.0
            marker_array.markers.append(m)
        if not frontiers:
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'frontiers'
            m.action = Marker.DELETEALL
            marker_array.markers.append(m)
        self.centroids_pub.publish(marker_array)

        cells = GridCells()
        cells.header.frame_id = 'map'
        cells.header.stamp = self.get_clock().now().to_msg()
        cells.cell_width = mapdata.info.resolution
        cells.cell_height = mapdata.info.resolution
        for f in frontiers:
            cells.cells.extend(f.cells)
        self.cells_pub.publish(cells)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
