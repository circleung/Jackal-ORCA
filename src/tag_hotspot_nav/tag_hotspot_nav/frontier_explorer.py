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
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32, String
from visualization_msgs.msg import Marker, MarkerArray
from slam_toolbox.srv import Reset
from tf2_ros import Buffer, TransformListener

from tag_hotspot_nav.frontier_detection import (
    MIN_FRONTIER_SIZE, detect_frontiers, passage_width_m, passage_depth_m,
    enclosure_ratio)
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
        # (B) 정보이득 식: frontier 크기(미탐색 경계 길이)가 클수록 비용 ↓ → 큰 빈
        #     영역을 우선 탐사하고 자잘한 틈을 촘촘히 훑지 않음. 0=크기 무시(순수 최근접).
        #     비용 /= (1 + size_weight·ln(size)). heading_factor 와 함께라 핑퐁 억제됨.
        self.declare_parameter('size_weight', 0.6)
        # 방문 영역 기억 → 재방문 차단: 로봇이 지나간 셀을 누적하고, 그 인근의
        # frontier 는 선택 비용에 ×visit_penalty 페널티 → 이미 훑은 영역 반복 대신
        # 새 영역 우선. 하드 스킵이 아니라 가중이라, 남은 게 그것뿐이면 결국 간다.
        self.declare_parameter('visit_penalty', 3.0)       # 1.0=off
        self.declare_parameter('visit_cell', 0.5)          # [m] 방문격자 해상도
        self.declare_parameter('visit_radius', 0.5)        # [m] 이 거리내 방문셀 있으면 페널티
        # 좁은 공간 무시: frontier 중심의 통로폭(벽-사이 최소폭)이 이 값 미만이면
        # frontier 생성 제외 → 좁은 방/통로를 반복 방문하느라 시간 낭비하는 것 방지.
        # 0=off. 벽이 max_range(=이 값+1m) 안에 양쪽 다 잡혀야 좁다고 판단(미탐색이면 일단 감).
        self.declare_parameter('min_passage_width', 1.5)   # [m]
        # 깊이 조건: 폭이 좁아도 '깊으면'(뒤에 방/공간) 유지 = 출입문 통과 보장.
        # 폭<min_passage_width AND 깊이<min_passage_depth 일 때만 제외(=좁고 얕은 포켓).
        self.declare_parameter('min_passage_depth', 2.0)   # [m] 0=깊이무시(폭만)
        # 좁고-얕음 스킵은 '멀리 있는' 포켓에만 적용. 이 거리 이내(=이미 진입/근처)면
        # 필터 안 함 → 들어간 골목은 끝까지 탐사(DFS). 멀리 있는 작은 포켓만 안 감.
        self.declare_parameter('passage_skip_dist', 2.0)   # [m]
        # goal 도달 후 AprilTag 스캔 회전 (front+back 카메라 합산 360° 커버)
        self.declare_parameter('scan_spin_duration', 7.0)   # [s] 7s × 0.5rad/s ≈ 200°
        self.declare_parameter('scan_spin_angular', 0.5)    # [rad/s]
        self.declare_parameter('cmd_vel_topic', '/j100_0915/cmd_vel')
        # 좁은 공간 탈출 후진
        self.declare_parameter('backup_duration', 2.5)      # [s]
        self.declare_parameter('backup_speed', -0.2)        # [m/s]
        # FIX-stuck-4: 후진 시 후방 장애물 인식 — 이 거리 이하로 막히면 후진 중단(벽 박기 방지)
        self.declare_parameter('backup_min_rear', 0.35)     # [m] 후방 최소 여유
        # FIX-stuck-2: 후진 후 제자리회전 탈출 — 모서리는 직진 후진만으론 같은 데로 재진입.
        self.declare_parameter('escape_rotate_duration', 1.5)  # [s] 0이면 회전 탈출 off
        self.declare_parameter('escape_rotate_speed', 0.6)     # [rad/s]
        # FIX-stuck-3: 끼임에 의한 조기 false-"탐사완료" 방지 — 종료 직전 탈출 시도 횟수
        self.declare_parameter('stuck_escape_max', 3)
        self.declare_parameter('blacklist_radius', 0.6)     # [m] 실패 frontier 회피 반경
        self.declare_parameter('blacklist_ttl', 60.0)       # [s] blacklist 유효 시간
        self.declare_parameter('blacklist_max_strikes', 3)  # 이 횟수 실패 시 영구 차단
        # 같은 자리(blacklist_radius 내) 목표에 "성공적으로" 도달한 횟수 제한.
        # 가구 등 고정 장애물에 가려진 포켓은 도달해도 못 밝혀져 매번 새 frontier로
        # 재등장 → 정상 도달은 blacklist 안 됨(166행) → 무한 재방문. 가구는 안 움직이므로
        # 같은 자리를 revisit_limit 회 넘게 갔으면 더 가도 소용없다고 보고 영구 차단.
        self.declare_parameter('revisit_limit', 2)
        # ── 진짜 DFS: 갈림길 분기 기억 + 백트래킹 ──────────────────
        # 후보 frontier 들이 현재 방향과 junction_angle_deg 이상 벌어진 별도
        # 방향으로 나뉘면 "갈림길"로 판단 → 안 택한 방향을 branch_stack 에 저장.
        # 현재 가지가 막히면(frontier 없음) 스택에서 꺼내 그 갈림길로 되돌아가
        # 다른 방향을 탐사 → 사용자가 원한 "한 갈래 끝까지 → 복귀 → 다음 갈래".
        self.declare_parameter('junction_angle_deg', 70.0)
        self.declare_parameter('max_branch_stack', 20)
        # 진짜 DFS: 멀리 계속 나가기 전에 "대기 중인 갈림길 중 가까운 것"을 먼저
        # 처리한다. 기존엔 현재 방향에 frontier 가 하나도 안 남아야만(가지 끝)
        # 스택을 꺼냈는데, 그 전까지 계속 멀리 나가며 갈림길이 쌓이기만 해서
        # 가까운 작은 방 구석이 한참 뒤로 밀리는 문제가 있었다. 매 사이클 가까운
        # 대기 갈림길이 있으면 우선 처리 → 시간도 적게 걸리고 누락도 줄어든다.
        self.declare_parameter('branch_revisit_dist', 5.0)  # [m]
        # 하드 방향 커밋: bias_bearing(고정 진행방향)에서 이 각도 이상 벗어난 frontier 는
        # 후보에서 완전히 제외(비용 가중 아님 — 무조건 배제). size_weight 할인이 커서
        # heading_weight 페널티를 이기고 반대편 큰 영역으로 튀어버리는 문제를 막는다.
        # 현재 방향에 후보가 하나도 없을 때만(가지 끝) 백트래킹 → 그래도 없으면 허용.
        self.declare_parameter('branch_commit_angle_deg', 100.0)
        # 유리방 등 "사방이 거의 막힌" frontier 자동 제외(좌표 불필요).
        # 유리는 벽 틀이 잡히지만 안쪽으로 투영된 free 셀이 새어나가 frontier 가
        # 생기는데, 진짜 출입구와 달리 거의 모든 방향에서 벽이 잡힌다.
        # enclosure_ratio_threshold 이상이면 제외(0=off).
        self.declare_parameter('enclosure_check_range', 3.0)     # [m]
        self.declare_parameter('enclosure_ratio_threshold', 0.75)  # 0=off, 0.75 권장
        # 방 진입 우선: '문'(좁은 입구 + 뒤에 깊은 공간=방) frontier 는 비용 ×doorway_bonus(<1)
        # 로 할인 → heading_weight(복도 DFS)는 그대로 두되, 옆방 문을 지나치기 전에 먼저 들름.
        # is_doorway = 통로폭 < doorway_max_width AND 깊이 >= doorway_min_depth.
        self.declare_parameter('doorway_bonus', 0.4)        # 1.0=off, 작을수록 방 우선(0.4=비용 40%)
        self.declare_parameter('doorway_max_width', 1.2)    # [m] 입구폭 이 미만이면 '문' 후보
        self.declare_parameter('doorway_min_depth', 2.0)    # [m] 뒤 공간 이 이상이면 '방'으로 인정
        # ── 병목 영구차단(유리방 등 진짜 못 지나가는 지점) ──────────────
        # 부분경로 끝점이 같은 자리(blacklist_radius 내)에 반복되는데, 그때마다
        # frontier 크기가 안 줄어들면(=진짜 전진이 없으면) chokepoint_strikes 회 누적 시
        # 그 끝점을 영구 차단. 크기가 충분히 줄면(=느려도 진짜 전진 중) 카운트 리셋 —
        # 작은 방을 조금씩 들어가는 정상적인 점진 접근(부분경로→맵 확장→재계획)은
        # 안 막는다. doorway_bonus/revisit_blacklist 와는 별개 메커니즘, 서로 안 건드림.
        self.declare_parameter('chokepoint_strikes', 3)
        self.declare_parameter('chokepoint_shrink_ratio', 0.85)  # 새 size <= 이전*이값 이어야 '진전'
        # ── 경로-겹침 페널티 (루프 왕복/이미 지나온 길 재추적 억제) ──────────
        # frontier 위치가 아니라 "거기까지 가는 A* 경로 자체가 이미 지나온 길(_visited)을
        # 얼마나 되짚는가"로 판단. 진짜 새 가지/방은 가는 길이 대부분 새 길이라 전혀
        # 안 걸리고, 큰 frontier 쫓아 반대편으로 튀거나 루프를 돌아가는 경우만 잡힌다.
        # (위치 반경 기반 차단과 달리 복도 끝까지 가는 정상 동작을 방해하지 않음)
        self.declare_parameter('path_overlap_threshold', 0.6)  # 경로의 이 비율 이상이 방문지면
        self.declare_parameter('path_overlap_penalty', 20.0)   # 비용에 곱할 페널티

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
        self.size_weight = self.get_parameter('size_weight').value
        self.visit_penalty = float(self.get_parameter('visit_penalty').value)
        self.visit_cell = float(self.get_parameter('visit_cell').value)
        self.visit_radius = float(self.get_parameter('visit_radius').value)
        self.min_passage_width = float(self.get_parameter('min_passage_width').value)
        self.min_passage_depth = float(self.get_parameter('min_passage_depth').value)
        self.passage_skip_dist = float(self.get_parameter('passage_skip_dist').value)
        self._visited = set()    # 방문 셀 {(ix,iy)} (visit_cell 해상도)
        self._visit_rad_cells = max(1, int(round(self.visit_radius / self.visit_cell)))
        self.scan_spin_duration = self.get_parameter('scan_spin_duration').value
        self.scan_spin_angular = self.get_parameter('scan_spin_angular').value
        self.backup_duration = self.get_parameter('backup_duration').value
        self.backup_speed = self.get_parameter('backup_speed').value
        self.escape_rotate_duration = self.get_parameter('escape_rotate_duration').value
        self.escape_rotate_speed = self.get_parameter('escape_rotate_speed').value
        self.stuck_escape_max = int(self.get_parameter('stuck_escape_max').value)
        self.backup_min_rear = float(self.get_parameter('backup_min_rear').value)
        self.rear_clear = float('inf')   # FIX-stuck-4: 후방 ±25° 최근접 거리(/scan)
        self._np_pos = None      # 무진전 감지: 마지막 진전 위치
        self._np_t = None
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.blacklist_ttl = self.get_parameter('blacklist_ttl').value
        self.blacklist_max_strikes = self.get_parameter('blacklist_max_strikes').value
        self.revisit_limit = self.get_parameter('revisit_limit').value
        self.junction_angle_deg = float(self.get_parameter('junction_angle_deg').value)
        self.max_branch_stack = int(self.get_parameter('max_branch_stack').value)
        self.branch_commit_angle_deg = float(self.get_parameter('branch_commit_angle_deg').value)
        self.branch_revisit_dist = float(self.get_parameter('branch_revisit_dist').value)
        self.enclosure_check_range = float(self.get_parameter('enclosure_check_range').value)
        self.enclosure_ratio_threshold = float(self.get_parameter('enclosure_ratio_threshold').value)
        self.doorway_bonus = float(self.get_parameter('doorway_bonus').value)
        self.doorway_max_width = float(self.get_parameter('doorway_max_width').value)
        self.doorway_min_depth = float(self.get_parameter('doorway_min_depth').value)
        self._doorway_cache = {}   # explore_step 마다 리셋되는 per-cycle 캐시 {id(f): factor}
        self.chokepoint_strikes = int(self.get_parameter('chokepoint_strikes').value)
        self.chokepoint_shrink_ratio = float(self.get_parameter('chokepoint_shrink_ratio').value)
        self._chokepoint_history = {}   # {(x,y): (last_size, strikes)} 부분경로 끝점 진전 추적
        self.chokepoint_blacklist = {}  # {(x,y): 차단 시각} 영구 차단된 병목 지점
        self._last_path_end = None      # 직전 발행 경로가 부분경로면 그 끝점
        self._last_frontier_size = None # 그 경로를 만든 frontier 크기(진전 판단용)
        self.path_overlap_threshold = float(self.get_parameter('path_overlap_threshold').value)
        self.path_overlap_penalty = float(self.get_parameter('path_overlap_penalty').value)

        # ── 입출력 ──────────────────────────────────────────────
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_reached_sub = self.create_subscription(
            Bool, '/goal_reached', self.goal_reached_callback, 10)
        self.command_sub = self.create_subscription(
            String, '/explore/command', self.command_callback, 10)
        # FIX-stuck-4: 후진 안전용 후방 거리 (BEST_EFFORT 로 /scan 매칭)
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, scan_qos)
        # DIAG: 미션 목표(apriltag 캡처) 추적 — /tag_new(신규 태그 이벤트) 카운트
        self._tags_captured = 0
        self.create_subscription(Int32, '/tag_new', self._tag_new_cb, 10)

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
        self._stuck_escape_count = 0 # FIX-stuck-3: 종료 직전 탈출 시도 횟수
        # 실패 frontier 기록 {(x, y): [마지막 실패 시각, 누적 실패 횟수]}
        # TTL 안에는 회피, TTL 지나면 재시도 자격 (도달 직후 일시 정체로
        # 영구 차단되는 문제 방지). max_strikes 누적 시에만 영구 차단.
        self.blacklist = {}
        # revisit_limit 으로 영구 차단된 자리. blacklist 와 분리해서 보관 —
        # try_final_sweep() 이 종료 직전 blacklist 를 비울 때(정상 동작) 이것까지
        # 같이 지워지면 "가구 등으로 막힌 자리"가 풀려 다시 가버린다(재방문 반복의 원인).
        self.revisit_blacklist = {}   # {(x, y): 차단된 시각} — 영구, final_sweep 에도 안 지워짐
        self.visit_counts = {}   # {(x, y): 성공 도달 횟수} — revisit_limit 판정용
        self.branch_stack = []   # [(x, y, bearing)] DFS 갈림길 — 안 가본 분기점 (백트래킹용)
        # 현재 가지의 "고정된 진행방향". None 이면 아직 미설정(첫 선택 시 로봇 yaw로 초기화).
        # 매 사이클 로봇의 순간 yaw 대신 이 값으로 heading_factor 를 계산해야,
        # 경로 도착 직후 방향이 틀어져도 "원래 가던 방향"을 계속 우선한다.
        # 완만한 커브는 따라가지만(점진 갱신) 급격한 반전은 안 따라간다(고정 유지)
        # → 그래서 가지 끝나기 전엔 반대편 frontier 로 안 튄다.
        self._branch_bearing = None
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

    def _tag_new_cb(self, msg: Int32):
        self._tags_captured += 1

    def scan_callback(self, msg: LaserScan):
        # FIX-stuck-4: 후방 ±25° 최근접 거리 → 후진 안전 게이트용
        rear = float('inf')
        half = math.radians(25.0)
        for i, r in enumerate(msg.ranges):
            if r <= msg.range_min or math.isinf(r) or math.isnan(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(abs(angle) - math.pi) < half:   # 후방(180°) ±25°
                rear = min(rear, r)
        self.rear_clear = rear

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
            # FIX-stuck-4: 후방이 backup_min_rear 이하로 막히면 후진 중단(벽 박기 방지)
            rear_ok = self.rear_clear > self.backup_min_rear
            if self._backup_end_time and now < self._backup_end_time and rear_ok:
                twist.twist.linear.x = self.backup_speed
            else:
                self._backing_up = False
                if not rear_ok:
                    self.get_logger().warn(
                        f'후방 {self.rear_clear:.2f}m 막힘 → 후진 중단(벽 박기 방지)')
                # FIX-stuck-2: 후진(또는 중단) 후 제자리회전으로 헤딩 전환
                if self.escape_rotate_duration > 0:
                    self._rotate_escape = True
                    self._rotate_end_time = (now + Duration(
                        seconds=self.escape_rotate_duration))
                    self.get_logger().info('후진 종료 → 제자리회전 탈출')
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
                    self.revisit_blacklist[(x, y)] = self.now_sec()
                    self.get_logger().warn(
                        f'같은 자리 ({x:.2f},{y:.2f}) {cnt}회 도달해도 새 영역 안 밝혀짐 '
                        '→ 고정 장애물(가구 등)로 판단, 영구 차단')
                return
        self.visit_counts[(cx, cy)] = 1
        # off-by-one 보정: 기존엔 '첫 방문'이 임계 검사를 건너뛰어 revisit_limit 1과 2가
        # 동일하게 동작(둘 다 2번째 도달에서 차단)했다. limit=1 이면 첫 도달에서 바로
        # 재방문 차단되도록 첫 방문도 검사한다. (먼 frontier 점진접근은 frontier 가
        # 후퇴하며 매번 >blacklist_radius 떨어진 새 지점이라 영향 적음)
        if 1 >= self.revisit_limit:
            self.revisit_blacklist[(cx, cy)] = self.now_sec()
            self.get_logger().info(
                f'방문지점 ({cx:.2f},{cy:.2f}) 도달 → 재방문 차단'
                f'(revisit_limit={self.revisit_limit})')

    def _record_chokepoint(self, ex: float, ey: float, size: int):
        """부분경로 끝점(병목)에서 frontier 크기가 줄지 않으면(=진전 없음) 누적,
        chokepoint_strikes 회 누적 시 그 끝점을 영구 차단. 크기가 충분히 줄면
        (=느려도 진짜 전진 중) 카운트 리셋 — 작은 방 점진 탐사는 안 막는다."""
        for (x, y), (last_size, strikes) in list(self._chokepoint_history.items()):
            if math.hypot(ex - x, ey - y) < self.blacklist_radius:
                if size <= last_size * self.chokepoint_shrink_ratio:
                    self._chokepoint_history[(x, y)] = (size, 0)   # 진전 → 리셋
                else:
                    strikes += 1
                    self._chokepoint_history[(x, y)] = (size, strikes)
                    if strikes >= self.chokepoint_strikes:
                        self.chokepoint_blacklist[(x, y)] = self.now_sec()
                        self.get_logger().warn(
                            f'경로 병목 ({x:.2f},{y:.2f}) {strikes}회 반복, frontier 크기 '
                            f'정체(size={size}) → 진짜 못 지나가는 곳으로 판단, 영구 차단')
                return
        self._chokepoint_history[(ex, ey)] = (size, 0)

    def _chokepoint_blocked(self, x: float, y: float) -> bool:
        for (bx, by) in self.chokepoint_blacklist:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        return False

    def goal_reached_callback(self, msg: Bool):
        if msg.data and self.is_navigating:
            self.is_navigating = False
            self._np_retry_goal = None
            if self.current_goal is not None:
                self._record_visit(*self.current_goal)
                # FIX-explore: 부분경로는 '먼 frontier 점진 접근'의 정상 단계다(A* 는
                # unknown 으로 경로를 못 내므로 먼 목표는 항상 부분경로로 시작 → 끝점
                # 도달 시 맵이 확장되어 다음엔 더 멀리 계획됨). 즉시 blacklist 하면
                # 도달가능한 먼 빈 영역을 한 번에 영구 포기 → 조기 종료/로컬미니멈의 주범.
                # 진짜 막다른 곳(같은 자리 반복, 새 영역 0)은 _record_visit 의
                # revisit_limit 이 차단하므로 여기선 blacklist 하지 않는다.
                if self._goal_partial:
                    self.get_logger().info(
                        f'부분경로 끝 도달 ({self.current_goal[0]:.2f},'
                        f'{self.current_goal[1]:.2f}) → 맵 확장, 재계획(blacklist 안 함)')
                    if self._last_path_end is not None and self._last_frontier_size is not None:
                        self._record_chokepoint(
                            self._last_path_end[0], self._last_path_end[1],
                            self._last_frontier_size)
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
            self.revisit_blacklist = {}
            self.visit_counts = {}
            self.branch_stack = []
            self._branch_bearing = None
            self._chokepoint_history = {}
            self.chokepoint_blacklist = {}
            self._last_path_end = None
            self._last_frontier_size = None
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

    def _size_factor(self, size):
        """frontier 크기에 따른 비용 할인 배율(<=1). 큰 영역일수록 비용 ↓ → 큰 빈
        영역 우선, 자잘한 틈 촘촘탐사 억제. 비용 *= size^(-size_weight).
        size_weight=0 크기무시, 0.7 권장, 1.0 은 순수 d/size(핑퐁 위험)."""
        if self.size_weight <= 0.0:
            return 1.0
        return float(max(size, 1)) ** (-self.size_weight)

    def _mark_visited(self, pose):
        """로봇 현재 위치를 방문 격자에 누적."""
        if pose is None:
            return
        self._visited.add((int(round(pose[0] / self.visit_cell)),
                           int(round(pose[1] / self.visit_cell))))

    def _visit_factor(self, fx, fy):
        """frontier 가 이미 방문한 영역 인근이면 비용 ×visit_penalty(>1). 멀면 1.0."""
        if self.visit_penalty <= 1.0 or not self._visited:
            return 1.0
        cx = int(round(fx / self.visit_cell))
        cy = int(round(fy / self.visit_cell))
        r = self._visit_rad_cells
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if (cx + dx, cy + dy) in self._visited:
                    return self.visit_penalty
        return 1.0

    def _path_overlap_ratio(self, path) -> float:
        """경로(Point 리스트) 중 이미 방문한 격자(_visited)에 속하는 비율(0~1).
        높으면 "목적지가 새 곳이어도 가는 길이 이미 지나온 길의 재추적"이라는 뜻."""
        if not path:
            return 0.0
        vc = self.visit_cell
        hits = 0
        for p in path:
            key = (int(round(p.x / vc)), int(round(p.y / vc)))
            if key in self._visited:
                hits += 1
        return hits / len(path)

    def _doorway_factor(self, f, robot_pose, mapdata):
        """'문'(좁은 입구 + 뒤에 깊은 공간=방) frontier 면 비용 보너스(<1) 반환 → 지나치기
        전에 먼저 진입. heading_weight(복도 DFS)는 유지하되 옆방 문만 끌어당긴다.
        passage_width/depth 재사용. 결과는 per-cycle 캐시(_doorway_cache)로 1회만 계산."""
        if self.doorway_bonus >= 1.0:
            return 1.0
        key = id(f)
        cached = self._doorway_cache.get(key)
        if cached is not None:
            return cached
        fgx, fgy = world_to_grid(mapdata, f.centroid.x, f.centroid.y)
        factor = 1.0
        w = passage_width_m(mapdata, fgx, fgy, self.doorway_max_width + 0.5)
        if w < self.doorway_max_width:   # 입구가 좁다 → 뒤 공간 깊이 확인
            dx = f.centroid.x - robot_pose[0]
            dy = f.centroid.y - robot_pose[1]
            depth = passage_depth_m(mapdata, fgx, fgy, dx, dy,
                                    self.doorway_min_depth + 2.0)
            if depth >= self.doorway_min_depth:   # 좁은 입구 + 깊은 공간 = 방 문
                factor = self.doorway_bonus
        self._doorway_cache[key] = factor
        return factor

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
            self._mark_visited(pose)   # 주행 중 방문 격자 누적
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
        self._mark_visited(robot_pose)   # 방문 격자 누적(재방문 차단용)
        mapdata = self.mapdata

        # 0) 진짜 DFS: 멀리 가기 전에 가까운 대기 갈림길부터 처리
        if self._try_nearby_branch(robot_pose, mapdata):
            return

        # 1) frontier 탐지
        start_grid = world_to_grid(mapdata, *robot_pose)
        frontiers_all = detect_frontiers(mapdata, start_grid, self.min_frontier_size)
        frontiers = [f for f in frontiers_all if not self.is_blacklisted(f.centroid)]
        self._doorway_cache = {}   # frontier 객체가 매 사이클 재생성되므로 id() 캐시 리셋 필수
        # 좁고 얕은 공간 frontier 제외(폭<min_passage_width AND 깊이<min_passage_depth)
        # → 좁은 포켓 반복방문 방지. 단 출입문(좁아도 뒤에 방=깊음)은 유지해 통과 보장.
        narrow = 0
        enclosed = 0
        if self.enclosure_ratio_threshold > 0.0 and frontiers:
            kept = []
            for f in frontiers:
                fgx, fgy = world_to_grid(mapdata, f.centroid.x, f.centroid.y)
                er = enclosure_ratio(mapdata, fgx, fgy, self.enclosure_check_range)
                if er >= self.enclosure_ratio_threshold:
                    enclosed += 1   # 사방이 거의 막힘 = 유리 안쪽 투영 등 가짜 의심 → 제외
                    continue
                kept.append(f)
            frontiers = kept
        if self.min_passage_width > 0.0 and frontiers:
            kept = []
            for f in frontiers:
                # 이미 근처(=진입했거나 가까운) frontier 는 필터 안 함 → 들어간 골목
                # 끝까지 탐사(DFS). 멀리 있는 작은 포켓만 스킵 대상.
                rdist = math.hypot(f.centroid.x - robot_pose[0],
                                   f.centroid.y - robot_pose[1])
                if rdist <= self.passage_skip_dist:
                    kept.append(f)
                    continue
                fgx, fgy = world_to_grid(mapdata, f.centroid.x, f.centroid.y)
                w = passage_width_m(mapdata, fgx, fgy, self.min_passage_width + 1.0)
                if w >= self.min_passage_width:
                    kept.append(f)
                    continue
                # 폭은 좁다 → 깊이 확인. 깊으면(출입문) 유지, 얕으면(포켓) 제외.
                if self.min_passage_depth > 0.0:
                    dx = f.centroid.x - robot_pose[0]
                    dy = f.centroid.y - robot_pose[1]
                    d = passage_depth_m(mapdata, fgx, fgy, dx, dy,
                                        self.min_passage_depth + 2.5)
                    if d >= self.min_passage_depth:
                        kept.append(f)   # 좁지만 깊다 = 출입문/통로 → 유지
                        continue
                narrow += 1   # 좁고 얕음 → 제외
            frontiers = kept
        self.publish_visualization(frontiers, mapdata)
        # DIAG: 실주행 최적화용 기록 — 매 사이클 탐지/후보/blacklist/태그 카운트.
        # (frontier 너무 적음=탐지문제 vs 후보만 적음=blacklist문제 즉시 구분)
        self.get_logger().info(
            f'[DIAG] frontier 탐지={len(frontiers_all)} 후보={len(frontiers)} '
            f'좁음제외={narrow} 막힘제외={enclosed} blacklist={len(self.blacklist)} '
            f'방문셀={len(self._visited)} 태그캡처={self._tags_captured}')

        # 2) 종료 판정
        if not frontiers:
            self.no_frontiers_count += 1
            self.get_logger().info(
                f'frontier 없음 ({self.no_frontiers_count}/{self.no_frontier_limit})')
            if self.no_frontiers_count >= self.no_frontier_limit:
                if self.try_final_sweep():
                    return
                if self._try_backtrack(robot_pose, mapdata):   # DFS: 갈림길로 복귀
                    return
                self.get_logger().info('탐사 완료!')
                self.finished = True
                self.plan_pub.publish(Path())          # 빈 경로 → 정지
                self.finish_pub.publish(Bool(data=True))
            return
        self.no_frontiers_count = 0

        # 3) 1차 선별 (싸게) → 상위 N 개만 A* 평가 (비싸게).
        #    (A) DFS 식: 전역 '큰 frontier' 선호(d/size)를 버리고, "고정된 진행방향"에
        #    가까운 frontier 를 우선 → 한 방향을 끝까지 파고듦(핑퐁 억제).
        #    ⚠ 로봇의 순간 yaw 가 아니라 _branch_bearing(가지 고정방향) 을 기준으로 삼는다.
        #    순간 yaw 를 쓰면 경로 도착 직후 방향이 틀어진 김에 반대편 frontier 로 튄다.
        ryaw = self._robot_yaw()   # 갈림길 감지(반대방향 판정)용 — 순간 yaw 그대로 사용
        bias_bearing = self._branch_bearing if self._branch_bearing is not None else ryaw

        def euclid_cost(f):
            d = math.hypot(f.centroid.x - robot_pose[0], f.centroid.y - robot_pose[1])
            if d < self.min_distance:
                return float('inf')
            return d * self._heading_factor(
                f.centroid.x, f.centroid.y, robot_pose[0], robot_pose[1], bias_bearing) \
                * self._size_factor(f.size) \
                * self._visit_factor(f.centroid.x, f.centroid.y) \
                * self._doorway_factor(f, robot_pose, mapdata)

        ranked = [f for f in sorted(frontiers, key=euclid_cost)
                  if euclid_cost(f) != float('inf')]
        if not ranked:
            return

        # ── DFS 하드 방향 커밋 ───────────────────────────────────
        # bias_bearing(고정 진행방향)에서 branch_commit_angle_deg 이상 벗어난
        # frontier 는 "비용 가중"이 아니라 후보에서 통째로 제외한다. size_weight
        # 할인이 커서 heading_weight 페널티만으로는 못 막는 경우(큰 반대편 영역으로
        # 튀는 것)를 하드 필터로 차단 — 현재 가지 안에 갈 곳이 남으면 무조건 그쪽부터.
        ranked_for_pick = ranked
        if bias_bearing is not None:
            in_cone, out_of_cone = [], []
            for f in ranked:
                fb = math.atan2(f.centroid.y - robot_pose[1], f.centroid.x - robot_pose[0])
                d = abs(math.atan2(math.sin(fb - bias_bearing), math.cos(fb - bias_bearing)))
                (in_cone if math.degrees(d) <= self.branch_commit_angle_deg
                 else out_of_cone).append(f)
            if in_cone:
                ranked_for_pick = in_cone
            else:
                # 현재 방향엔 더 갈 곳 없음 = 가지 끝 → 갈림길로 백트래킹 우선
                if self._try_backtrack(robot_pose, mapdata):
                    return
                # 백트래킹도 안 되면(스택 없음/다 막힘) 마지막 수단으로 반대편 허용
                ranked_for_pick = out_of_cone if out_of_cone else ranked

        candidates = ranked_for_pick[:self.top_n_astar]
        if not candidates:
            return

        # ── DFS 갈림길 감지 ──────────────────────────────────────
        # 가장 우선(가까운+진행방향) 후보와 junction_angle_deg 이상 벌어진 별도
        # 방향의 후보가 있으면 "갈림길"로 보고, 안 택한 방향을 branch_stack 에 저장.
        # 나중에 현재 가지가 막히면 여기로 되돌아가 그 방향을 탐사한다.
        if ryaw is not None and len(ranked_for_pick) >= 2 and len(self.branch_stack) < self.max_branch_stack:
            primary = ranked_for_pick[0]
            p_bearing = math.atan2(primary.centroid.y - robot_pose[1],
                                   primary.centroid.x - robot_pose[0])
            for alt in ranked_for_pick[1:self.top_n_astar]:
                a_bearing = math.atan2(alt.centroid.y - robot_pose[1],
                                       alt.centroid.x - robot_pose[0])
                diff = abs(math.atan2(math.sin(a_bearing - p_bearing),
                                      math.cos(a_bearing - p_bearing)))
                if math.degrees(diff) >= self.junction_angle_deg:
                    dup = any(math.hypot(alt.centroid.x - bx, alt.centroid.y - by)
                             < self.blacklist_radius for bx, by, _ in self.branch_stack)
                    if not dup:
                        self.branch_stack.append((alt.centroid.x, alt.centroid.y, a_bearing))
                        self.get_logger().info(
                            f'갈림길 감지 → 다른 방향 ({alt.centroid.x:.2f},'
                            f'{alt.centroid.y:.2f}) 저장 (대기 중 갈림길 '
                            f'{len(self.branch_stack)}개), 주방향 먼저 탐사')
                    break

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
            # 영구 차단된 병목(반복 정체 확인됨)을 지나야 하면 후보 제외
            if not reached and self._chokepoint_blocked(path[-1].x, path[-1].y):
                continue
            # (A)DFS 진행방향 가중 × (B)정보이득 크기 할인 → 큰 빈 영역 우선, 자잘틈 억제
            cost *= self._heading_factor(
                f.centroid.x, f.centroid.y, robot_pose[0], robot_pose[1], bias_bearing)
            cost *= self._size_factor(f.size)
            cost *= self._visit_factor(f.centroid.x, f.centroid.y)
            cost *= self._doorway_factor(f, robot_pose, mapdata)
            # 목적지 위치가 아니라 "가는 길 자체"가 이미 지나온 길을 되짚으면 페널티.
            # 큰 frontier 가 반대편(이미 지나온 루프 너머)에 있어도, 거기 가는 경로가
            # 대부분 새 길이면(=진짜 갈림길/방) 전혀 안 걸리고, 루프를 되돌아가는
            # 경우만 정확히 잡힌다.
            if self._path_overlap_ratio(path) >= self.path_overlap_threshold:
                cost *= self.path_overlap_penalty
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
                if not reached and self._chokepoint_blocked(path[-1].x, path[-1].y):
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
                # FIX-stuck-3: 끼임에 의한 조기 false-완료 방지 — 종료 직전 탈출 시도(K회).
                # 어디로도 경로계획이 지속 실패 = 끼임 가능성 → 후진/회전으로 재위치 후
                # 재시도. K회 모두 소용없을 때만 진짜 종료.
                if (self._stuck_escape_count < self.stuck_escape_max
                        and self.backup_duration > 0):
                    self._stuck_escape_count += 1
                    self._backing_up = True
                    self._backup_end_time = (self.get_clock().now()
                                             + Duration(seconds=self.backup_duration))
                    self.is_navigating = False
                    self.all_failed_count = 0   # 종료 보류, 재시도
                    self.get_logger().warn(
                        f'진행 불가 지속 → 끼임 탈출 시도 '
                        f'({self._stuck_escape_count}/{self.stuck_escape_max}) 후 재시도')
                    return
                if self._try_backtrack(robot_pose, mapdata):   # DFS: 갈림길로 복귀
                    return
                self.get_logger().info('진행 가능한 frontier 없음 → 탐사 완료!')
                self.finished = True
                self.plan_pub.publish(Path())
                self.finish_pub.publish(Bool(data=True))
            return
        self.all_failed_count = 0
        self._stuck_escape_count = 0    # 진행 재개 → 탈출 시도 카운트 리셋
        self.final_sweep_done = False   # 진행 재개 → 다음 정체 때 스윕 재허용
        # 성공한 사이클의 실패 후보만 blacklist 에 기록 (TTL + strike 방식)
        for cx, cy in cycle_blacklist:
            self.add_to_blacklist(cx, cy)

        # 4) 경로 발행
        self.publish_plan(best_path)
        self.is_navigating = True
        # 가지 고정방향 갱신: 완만한 커브(<100°)는 따라가며 갱신, 급격한 반전이면
        # (=다른 가지로 잘못 튄 것) 고정값을 유지해 다음 사이클에 다시 안 끌려가게 한다.
        new_bearing = math.atan2(best_frontier.centroid.y - robot_pose[1],
                                 best_frontier.centroid.x - robot_pose[0])
        if self._branch_bearing is None:
            self._branch_bearing = new_bearing
        else:
            bdiff = abs(math.atan2(math.sin(new_bearing - self._branch_bearing),
                                   math.cos(new_bearing - self._branch_bearing)))
            if math.degrees(bdiff) < 100.0:
                self._branch_bearing = new_bearing
        # 막힘(goal_timeout·무진전) 시 blacklist 대상이 될 현재 목표 기록.
        # (이게 None 이면 blacklist 분기가 죽어 같은 frontier 를 무한 재선택함)
        self.current_goal = (best_frontier.centroid.x, best_frontier.centroid.y)
        # 부분경로(도달 불가) 목표는 끝점에 "도달"해도 그 영역이 안 밝혀지므로,
        # 도달 시 성공으로 치지 말고 blacklist 해야 같은 frontier 무한 재선택을 끊는다.
        self._goal_partial = not best_reached
        if self._goal_partial:
            self._last_path_end = (best_path[-1].x, best_path[-1].y)
            self._last_frontier_size = best_frontier.size
        else:
            self._last_path_end = None
            self._last_frontier_size = None
        # 재시도 추적: 다른 목표로 넘어갔으면 무진전 재시도 상태 초기화
        if (self._np_retry_goal is not None and
                math.hypot(self.current_goal[0] - self._np_retry_goal[0],
                           self.current_goal[1] - self._np_retry_goal[1])
                >= self.blacklist_radius):
            self._np_retry_goal = None
        self.goal_start_time = self.get_clock().now()
        is_door = self._doorway_factor(best_frontier, robot_pose, mapdata) < 1.0
        self.get_logger().info(
            f'frontier 목표 ({best_frontier.centroid.x:.2f}, '
            f'{best_frontier.centroid.y:.2f}) size={best_frontier.size}, '
            f'{len(best_path)} waypoints{" [문→방 진입]" if is_door else ""}')

    def _try_nearby_branch(self, robot_pose, mapdata) -> bool:
        """진짜 DFS: 다음 목표를 정하기 전에, 대기 중인 갈림길 중 branch_revisit_dist
        안에 있는 가장 가까운 것이 있으면 먼저 처리한다(거리 기준 — 멀리 있는 갈림길은
        그대로 스택에 남아 나중에 _try_backtrack 이 처리). 성공 시 True."""
        if not self.branch_stack:
            return False
        near_i, near_d = None, self.branch_revisit_dist
        for i, (bx, by, _b) in enumerate(self.branch_stack):
            d = math.hypot(bx - robot_pose[0], by - robot_pose[1])
            if d < near_d:
                near_d, near_i = d, i
        if near_i is None:
            return False
        bx, by, bearing = self.branch_stack.pop(near_i)
        if self._point_blacklisted(bx, by):
            return False
        planner = PathPlanner(mapdata, robot_radius=self.robot_radius)
        path, cost, reached = planner.plan(robot_pose, (bx, by), truncate_end_cells=0)
        if path is None or math.hypot(path[-1].x - robot_pose[0],
                                      path[-1].y - robot_pose[1]) < 0.3:
            return False
        self.get_logger().info(
            f'근처 대기 갈림길 우선 처리 ({bx:.2f},{by:.2f}, {near_d:.1f}m) '
            f'(남은 갈림길 {len(self.branch_stack)}개)')
        self.publish_plan(path)
        self.is_navigating = True
        self.current_goal = (bx, by)
        self._goal_partial = not reached
        self._last_path_end = (path[-1].x, path[-1].y) if self._goal_partial else None
        self._last_frontier_size = None   # 갈림길 목표는 frontier size 정보 없음(체크포인트 추적 skip)
        self.goal_start_time = self.get_clock().now()
        self.no_frontiers_count = 0
        self.all_failed_count = 0
        self._branch_bearing = bearing
        return True

    def _try_backtrack(self, robot_pose, mapdata) -> bool:
        """현재 가지가 막혔을 때 DFS 백트래킹: branch_stack 에서 안 가본 갈림길로
        복귀 시도. 성공(경로 발행)하면 True, 더 갈 곳 없으면 False(호출자가 종료 판단)."""
        while self.branch_stack:
            bx, by, bearing = self.branch_stack.pop()
            if self._point_blacklisted(bx, by):
                continue   # 그 사이 다른 이유로 막힌 곳이면 스킵
            planner = PathPlanner(mapdata, robot_radius=self.robot_radius)
            path, cost, reached = planner.plan(robot_pose, (bx, by), truncate_end_cells=0)
            if path is None or math.hypot(path[-1].x - robot_pose[0],
                                          path[-1].y - robot_pose[1]) < 0.3:
                continue
            self.get_logger().info(
                f'현재 가지 막힘 → 갈림길 백트래킹 ({bx:.2f},{by:.2f}) '
                f'(대기 중 갈림길 {len(self.branch_stack)}개)')
            self.publish_plan(path)
            self.is_navigating = True
            self.current_goal = (bx, by)
            self._goal_partial = not reached
            self.goal_start_time = self.get_clock().now()
            self.no_frontiers_count = 0
            self.all_failed_count = 0
            self._branch_bearing = bearing   # 새 가지 방향으로 고정값 전환
            return True
        return False

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
        # TTL 임시 차단만 초기화 — strikes 가 영구차단 기준(blacklist_max_strikes)을
        # 채운 항목은 보존한다. 예전엔 전부 비워서 "60초 타임아웃 → 영구 차단"된
        # 자리도 매번 재검토 때마다 부활해 같은 곳을 무한 재시도하는 버그가 있었다.
        self.blacklist = {
            k: v for k, v in self.blacklist.items()
            if v[1] >= self.blacklist_max_strikes
        }
        # revisit_blacklist(가구 등으로 영영 못 밝히는 자리)는 절대 안 지운다 —
        # 여기까지 같이 비우면 "재방문 영구차단"이 막힐 때마다 풀려서 또 가버린다.
        self.no_frontiers_count = 0
        self.all_failed_count = 0
        self.get_logger().info('종료 전 최종 재확인 — blacklist 초기화 후 전체 재평가')
        return True

    def is_blacklisted(self, centroid) -> bool:
        for (x, y) in self.revisit_blacklist:   # 재방문 차단 — 영구, TTL 없음
            if math.hypot(centroid.x - x, centroid.y - y) < self.blacklist_radius:
                return True
        now = self.now_sec()
        for (x, y), (stamp, strikes) in self.blacklist.items():
            if math.hypot(centroid.x - x, centroid.y - y) < self.blacklist_radius:
                if strikes >= self.blacklist_max_strikes:   # 영구 차단
                    return True
                if now - stamp < self.blacklist_ttl:        # 아직 TTL 내
                    return True
        return False

    def _point_blacklisted(self, x: float, y: float) -> bool:
        """branch_stack 백트래킹 좌표용 — Point 객체 없이 좌표만으로 차단 여부 확인."""
        for (bx, by) in self.revisit_blacklist:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        now = self.now_sec()
        for (bx, by), (stamp, strikes) in self.blacklist.items():
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                if strikes >= self.blacklist_max_strikes:
                    return True
                if now - stamp < self.blacklist_ttl:
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
