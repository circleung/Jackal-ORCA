"""finish_mission_listener.py — /finish_exploration 신호를 계속 대기하다가,
(자동 종료든 사람이 수동으로 보낸 신호든 상관없이) 신호가 오면 자동으로:
  1) frontier_explorer 종료
  2) 기록된 태그 좌표 평균(밀집 중심) 계산
  3) 맵 상 실제 정차 가능한 가장 가까운 지점으로 스냅
  4) 그 지점 1곳으로 경로 계산 후 이동
  5) 도착 확인 → 완료음

hotspot_navigator(여러 hotspot 순회)는 launch 에서 auto_start:=false 로 꺼서
이 노드와 안 겹치게 한다. 이 노드는 계속 떠서 신호를 기다리는 상시 노드.

사용법 (start_explore 가 자동 기동함. 수동 실행 시):
  ssh jackal
  source /opt/ros/jazzy/setup.bash && source ~/colcon_ws/install/setup.bash
  python3 ~/colcon_ws/src/tag_hotspot_nav/scripts/finish_mission_listener.py &
"""
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
from custom_msgs.msg import TagPoseArray
from tf2_ros import Buffer, TransformListener
import rclpy.time

from tag_hotspot_nav.path_planner import PathPlanner
from tag_hotspot_nav.grid_utils import world_to_grid, grid_to_world


class FinishMissionListener(Node):
    def __init__(self):
        super().__init__('finish_mission_listener')
        self._mapdata = None
        self._tags = None
        self._done = False   # 한 세션에 한 번만 동작(중복 트리거 방지)

        self.create_subscription(OccupancyGrid, '/map', self._on_map, 1)
        self.create_subscription(TagPoseArray, '/tags_in_map', self._on_tags, 10)
        self.create_subscription(Bool, '/finish_exploration', self._on_finish, 10)
        self.create_subscription(Bool, '/goal_reached', self._on_goal_reached, 10)

        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        self.final_pub = self.create_publisher(Bool, '/final_goal_reached', 10)

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        self._goal_reached_event = threading.Event()
        self.get_logger().info(
            'finish_mission_listener 대기 중 — /finish_exploration 오면 '
            '태그 평균 지점으로 자동 이동합니다.')

    def _on_map(self, msg):
        self._mapdata = msg

    def _on_tags(self, msg):
        self._tags = msg

    def _on_goal_reached(self, msg):
        if msg.data:
            self._goal_reached_event.set()

    def _kill_frontier_explorer(self):
        out = subprocess.run(
            ['pgrep', '-f', 'lib/tag_hotspot_nav/frontier_explorer'],
            capture_output=True, text=True).stdout.strip()
        pids = [p for p in out.split('\n') if p]
        for pid in pids:
            subprocess.run(['kill', '-9', pid])
        if pids:
            self.get_logger().info(f'frontier_explorer 종료함 (pid {pids})')

    def _on_finish(self, msg):
        # 콜백은 메인 스핀 스레드에서 실행되므로 여기서 절대 rclpy.spin_once/spin
        # 을 다시 호출하면 안 됨("Executor is already spinning" 충돌). 시간 걸리는
        # 작업은 별도 스레드로 넘기고, 메인 스레드는 계속 다른 토픽(TF 등)을 받게 둔다.
        if not msg.data or self._done:
            return
        self._done = True
        threading.Thread(target=self._run_mission, daemon=True).start()

    def _run_mission(self):
        self.get_logger().info('탐사 종료 신호 수신 → 태그 평균 지점으로 이동 시작')
        self._kill_frontier_explorer()

        if self._tags is None or not self._tags.tags:
            self.get_logger().warn('태그가 없어서 이동 안 함')
            return
        if self._mapdata is None:
            self.get_logger().warn('맵이 없어서 이동 안 함')
            return

        xs = [t.pose.pose.position.x for t in self._tags.tags]
        ys = [t.pose.pose.position.y for t in self._tags.tags]
        avg_x, avg_y = sum(xs) / len(xs), sum(ys) / len(ys)
        self.get_logger().info(f'태그 {len(xs)}개 평균: ({avg_x:.2f}, {avg_y:.2f})')

        planner = PathPlanner(self._mapdata, robot_radius=0.25)
        target_grid = world_to_grid(self._mapdata, avg_x, avg_y)
        snapped = planner.nearest_walkable(target_grid)
        if snapped is None:
            self.get_logger().warn('정차 가능한 지점을 못 찾음')
            return
        sx, sy = (grid_to_world(self._mapdata, snapped).x,
                  grid_to_world(self._mapdata, snapped).y)
        self.get_logger().info(f'정차 가능 지점으로 스냅: ({sx:.2f}, {sy:.2f})')

        pose = None
        t0 = time.time()
        while pose is None and time.time() - t0 < 6:
            try:
                tfm = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time())
                pose = (tfm.transform.translation.x, tfm.transform.translation.y)
            except Exception:
                time.sleep(0.2)
        if pose is None:
            self.get_logger().warn('로봇 위치를 못 가져옴')
            return

        path, cost, reached = planner.plan(pose, (sx, sy), truncate_end_cells=0)
        if path is None:
            self.get_logger().warn('경로 계획 실패')
            return

        out_msg = Path()
        out_msg.header.frame_id = 'map'
        out_msg.header.stamp = self.get_clock().now().to_msg()
        for p in path:
            ps = PoseStamped()
            ps.header = out_msg.header
            ps.pose.position = p
            ps.pose.orientation = Quaternion(w=1.0)
            out_msg.poses.append(ps)
        self.plan_pub.publish(out_msg)
        self.get_logger().info(f'경로 발행({len(path)} waypoints) — 이동 시작')

        self._goal_reached_event.clear()
        self._goal_reached_event.wait(timeout=180)

        self.final_pub.publish(Bool(data=True))
        self.get_logger().info('도착 → 완료음 신호 발행. 미션 종료.')


def main():
    rclpy.init(args=['--ros-args', '-r', '/tf:=/j100_0915/tf',
                     '-r', '/tf_static:=/j100_0915/tf_static'])
    node = FinishMissionListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
