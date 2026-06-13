"""
hotspot_navigator.py — hotspot 들을 순서대로 접근하는 시퀀서 FSM (4단계).

탐사(Phase 1~2)가 끝나면 clustering(Phase 3)이 낸 /hotspots(밀집순 PoseArray)를
받아, 각 hotspot 중심으로 A* 경로계획 → pure_pursuit 주행 → 도착 확인을 반복한다.
주행 인프라는 기존 것을 그대로 재사용한다:
  - 경로계획: path_planner.PathPlanner (frontier_explorer 와 동일한 A*/C-space)
  - 추종:     pure_pursuit (이 노드가 내는 /plan 을 구독, 도착 시 /goal_reached 발행)

frontier_explorer 와 /plan·/goal_reached 를 공유하지만, frontier_explorer 는
탐사 종료(finished) 후 /plan 발행을 멈추므로 /finish_exploration 이후에만
이 노드가 동작하면 충돌하지 않는다.

상태(FSM):
  IDLE        → 트리거(/finish_exploration[auto_start] 또는 'approach' 명령) → APPROACHING
  APPROACHING → /goal_reached → DWELL
              → goal_timeout/무진전 → 재계획(재시도 한도 초과 시 다음 hotspot)
  DWELL       → dwell_sec 경과 → 다음 hotspot (APPROACHING) / 마지막이면 DONE
  DONE        → /final_goal_reached 발행 (safety_layer 자동 정지)
  (모든 상태) → 'pause' → 동결 / 'resume' → 재개 / 'reset' → IDLE 로 초기화

트리거:
  - auto_start=True(기본): /finish_exploration 수신 시 자동 시작 (완전 자율).
  - 'approach' (/explore/command): 수동 시작 (저장 후 직접 트리거하고 싶을 때).

목표 좌표 주의:
  hotspot 중심은 '벽에 붙은 태그들'의 평균이라 보통 벽 안/근처(통행 불가)다.
  PathPlanner.nearest_walkable 이 가장 가까운 통행가능 셀로 스냅하므로,
  로봇은 hotspot 에 '갈 수 있는 만큼 가깝게' 접근한 뒤 멈춘다.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseArray, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from tag_hotspot_nav.grid_utils import world_to_grid
from tag_hotspot_nav.path_planner import PathPlanner


class HotspotNavigatorNode(Node):

    def __init__(self):
        super().__init__('hotspot_navigator')

        # ── 파라미터 ─────────────────────────────────────────────
        self.declare_parameter('robot_radius', 0.25)        # [m] C-space 팽창 (frontier 와 동일)
        self.declare_parameter('map_topic', '/map')         # 접근 단계 맵 (정리맵 쓰려면 /map_nav)
        self.declare_parameter('auto_start', True)          # /finish_exploration 시 자동 시작
        self.declare_parameter('dwell_sec', 2.0)            # [s] hotspot 도착 후 정지 관찰
        self.declare_parameter('goal_timeout', 40.0)        # [s] 한 hotspot 접근 제한시간
        self.declare_parameter('no_progress_timeout', 10.0) # [s] 무진전 → 재계획
        self.declare_parameter('no_progress_dist', 0.15)    # [m] 진전 판정 거리
        self.declare_parameter('max_retries', 2)            # hotspot 당 재계획 한도(초과 시 스킵)
        self.declare_parameter('arrive_tolerance', 0.4)     # [m] 스냅목표 근방이면 도착 간주(보조)

        self.robot_radius = self.get_parameter('robot_radius').value
        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.dwell_sec = float(self.get_parameter('dwell_sec').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout').value)
        self.no_progress_timeout = float(self.get_parameter('no_progress_timeout').value)
        self.no_progress_dist = float(self.get_parameter('no_progress_dist').value)
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.arrive_tolerance = float(self.get_parameter('arrive_tolerance').value)
        map_topic = self.get_parameter('map_topic').value

        # ── 입출력 ──────────────────────────────────────────────
        self.create_subscription(OccupancyGrid, map_topic, self.map_cb, 10)
        self.create_subscription(PoseArray, '/hotspots', self.hotspots_cb, 10)
        self.create_subscription(Bool, '/finish_exploration', self.finish_cb, 10)
        self.create_subscription(Bool, '/goal_reached', self.goal_reached_cb, 10)
        self.create_subscription(String, '/explore/command', self.command_cb, 10)

        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        self.final_pub = self.create_publisher(Bool, '/final_goal_reached', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── 상태 ────────────────────────────────────────────────
        self.state = 'IDLE'
        self.mapdata = None
        self.hotspots = []          # [(x, y)] 밀집순 (clustering 발행 순서 유지)
        self.idx = 0                # 현재 접근 중인 hotspot 인덱스
        self.retries = 0
        self.reached = False        # /goal_reached 수신 플래그 (타이머에서 소비)
        self.goal_start = None
        self.dwell_start = None
        self.paused = False
        self._np_pos = None         # 무진전 감지 기준 위치
        self._np_t = None

        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(
            f'hotspot_navigator 대기 — auto_start={self.auto_start} '
            f"(수동 시작: /explore/command 'approach')")

    # ── 콜백 ────────────────────────────────────────────────────
    def map_cb(self, msg: OccupancyGrid):
        self.mapdata = msg

    def hotspots_cb(self, msg: PoseArray):
        # 접근 중이 아닐 때만 목표 목록 갱신(주행 중 목록이 바뀌어 인덱스가 어긋나는 것 방지)
        if self.state in ('IDLE', 'DONE'):
            self.hotspots = [(p.position.x, p.position.y) for p in msg.poses]

    def finish_cb(self, msg: Bool):
        if msg.data and self.auto_start and self.state == 'IDLE':
            self.start_mission('탐사 종료(/finish_exploration) 자동 시작')

    def goal_reached_cb(self, msg: Bool):
        if msg.data and self.state == 'APPROACHING':
            self.reached = True

    def command_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'approach':
            if self.state in ('IDLE', 'DONE'):
                self.start_mission("명령 'approach' 수동 시작")
        elif cmd == 'pause':
            if not self.paused:
                self.paused = True
                self.get_logger().info("명령 'pause' → 접근 동결")
        elif cmd in ('go', 'resume'):
            if self.paused:
                self.paused = False
                self._np_pos = None      # 재개 시 무진전 타이머 리셋
                if self.state == 'APPROACHING':
                    self.plan_to_current()   # 정지해 있던 pure_pursuit 에 경로 재투입
                self.get_logger().info(f"명령 '{cmd}' → 접근 재개")
        elif cmd == 'reset':
            self.abort('reset')

    # ── 미션 제어 ───────────────────────────────────────────────
    def start_mission(self, reason: str):
        if not self.hotspots:
            self.get_logger().warn(f'{reason} — 그러나 /hotspots 가 비어 있음. 대기.')
            return
        self.idx = 0
        self.retries = 0
        self.paused = False
        self.get_logger().info(f'{reason} — hotspot {len(self.hotspots)}개 접근 시작')
        self.enter_approaching()

    def abort(self, reason: str):
        self.plan_pub.publish(Path())     # 빈 경로 → pure_pursuit 정지
        self.state = 'IDLE'
        self.idx = 0
        self.retries = 0
        self.reached = False
        self._np_pos = None
        self.get_logger().info(f"'{reason}' → 접근 중단, IDLE")

    def enter_approaching(self):
        self.state = 'APPROACHING'
        self.reached = False
        self.retries = 0
        self._np_pos = None
        self.goal_start = self.get_clock().now()
        if not self.plan_to_current():
            # 계획 실패 → 다음 hotspot 로
            self.advance('경로계획 실패')

    # ── 메인 루프 ──────────────────────────────────────────────
    def tick(self):
        if self.paused or self.state in ('IDLE', 'DONE'):
            return

        if self.state == 'DWELL':
            if (self.get_clock().now() - self.dwell_start).nanoseconds * 1e-9 >= self.dwell_sec:
                self.advance('dwell 종료')
            return

        # state == 'APPROACHING'
        if self.reached:
            self.get_logger().info(
                f'hotspot #{self.idx} 도착 → {self.dwell_sec:.0f}s 관찰')
            self.state = 'DWELL'
            self.dwell_start = self.get_clock().now()
            self.plan_pub.publish(Path())   # 정지
            return

        now = self.get_clock().now()
        elapsed = (now - self.goal_start).nanoseconds * 1e-9
        if elapsed > self.goal_timeout:
            self.retry_or_skip(f'타임아웃({self.goal_timeout:.0f}s)')
            return

        # 무진전(막힘) 감지 → 재계획
        pose = self.get_robot_pose()
        tnow = now.nanoseconds * 1e-9
        if pose is not None:
            if (self._np_pos is None or
                    math.hypot(pose[0] - self._np_pos[0],
                               pose[1] - self._np_pos[1]) > self.no_progress_dist):
                self._np_pos = pose
                self._np_t = tnow
            elif tnow - self._np_t > self.no_progress_timeout:
                self.retry_or_skip(f'무진전({self.no_progress_timeout:.0f}s)')

    def retry_or_skip(self, why: str):
        self.retries += 1
        if self.retries > self.max_retries:
            self.get_logger().warn(
                f'hotspot #{self.idx} {why} — 재시도 한도 초과, 스킵')
            self.advance('스킵')
            return
        self.get_logger().warn(
            f'hotspot #{self.idx} {why} — 재계획 ({self.retries}/{self.max_retries})')
        self.goal_start = self.get_clock().now()
        self._np_pos = None
        if not self.plan_to_current():
            self.advance('재계획 실패')

    def advance(self, reason: str):
        self.idx += 1
        if self.idx >= len(self.hotspots):
            self.get_logger().info(f'모든 hotspot 접근 완료 ({reason}) → 미션 종료')
            self.state = 'DONE'
            self.plan_pub.publish(Path())
            self.final_pub.publish(Bool(data=True))   # safety_layer 자동 정지
            return
        self.enter_approaching()

    # ── 경로계획 ───────────────────────────────────────────────
    def plan_to_current(self) -> bool:
        """현재 hotspot 으로 A* 경로 계획 후 /plan 발행. 성공 여부 반환."""
        if self.mapdata is None:
            self.get_logger().warn('맵 미수신 — 경로계획 보류')
            return False
        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().warn('TF map→base_link 대기 중', throttle_duration_sec=5.0)
            return False

        goal = self.hotspots[self.idx]
        planner = PathPlanner(self.mapdata, robot_radius=self.robot_radius)
        start_grid = world_to_grid(self.mapdata, pose[0], pose[1])
        if planner.nearest_walkable(start_grid) is None:
            self.get_logger().warn('로봇 위치 C-space 스냅 실패 — 다음 tick 재시도')
            return False

        path, _cost = planner.plan(pose, goal)
        if path is None or len(path) < 2:
            self.get_logger().warn(
                f'hotspot #{self.idx} ({goal[0]:.2f},{goal[1]:.2f}) 경로 없음')
            return False

        self.publish_plan(path)
        self.get_logger().info(
            f'hotspot #{self.idx}/{len(self.hotspots) - 1} '
            f'({goal[0]:.2f},{goal[1]:.2f}) 접근 — {len(path)} waypoints')
        return True

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

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            tr = t.transform.translation
            return tr.x, tr.y
        except Exception:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = HotspotNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
