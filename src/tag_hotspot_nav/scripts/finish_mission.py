"""finish_mission.py — 탐사를 "이 정도면 됐다"고 판단했을 때 실행하는 단일 스크립트.

절차 (전부 이 스크립트 하나에서 순서대로 처리):
  1) frontier_explorer 프로세스 종료 (더 이상 새 탐사 경로를 만들지 않게)
  2) /tags_in_map 에서 기록된 태그 좌표들을 읽어 평균(밀집 중심) 계산
  3) 그 평균 지점을 맵 상 실제 정차 가능한(walkable) 가장 가까운 지점으로 스냅
  4) 로봇 현재 위치 → 그 지점까지 A* 경로 계산 후 /plan 1회 발행 (pure_pursuit 가 추종)
  5) /goal_reached 가 올라올 때까지 대기(도착 확인)
  6) /final_goal_reached 발행 → sound_player 가 완료음 재생 + (safety_layer 켜져있으면 자동 정지)

사용법 (Jackal 에서):
  ssh jackal
  source /opt/ros/jazzy/setup.bash && source ~/colcon_ws/install/setup.bash
  python3 ~/colcon_ws/src/tag_hotspot_nav/scripts/finish_mission.py
"""
import subprocess
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


def kill_frontier_explorer():
    """frontier_explorer 프로세스를 찾아서 종료. pause 명령은 pure_pursuit/
    hotspot_navigator 도 같이 멈춰서(같은 /explore/command 토픽 공유) 안 쓴다."""
    out = subprocess.run(
        ['pgrep', '-f', 'lib/tag_hotspot_nav/frontier_explorer'],
        capture_output=True, text=True).stdout.strip()
    pids = [p for p in out.split('\n') if p]
    for pid in pids:
        subprocess.run(['kill', '-9', pid])
    if pids:
        print(f'[1/6] frontier_explorer 종료함 (pid {pids})')
    else:
        print('[1/6] frontier_explorer 이미 안 떠있음')
    time.sleep(1.0)


def restart_sound_player():
    """sound_player 는 완료음을 세션당 1회만 재생하는 래치가 있다.
    이전에 한 번이라도 울렸으면 재시작해서 래치를 풀어준다."""
    out = subprocess.run(['pgrep', '-f', 'lib/tag_hotspot_nav/sound_player'],
                         capture_output=True, text=True).stdout.strip()
    pids = [p for p in out.split('\n') if p]
    for pid in pids:
        subprocess.run(['kill', '-9', pid])
    time.sleep(1.0)
    subprocess.Popen(
        ['bash', '-c',
         'source /opt/ros/jazzy/setup.bash && '
         'source ~/colcon_ws/install/setup.bash && '
         'ros2 run tag_hotspot_nav sound_player'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)
    print('[0/6] sound_player 재시작(완료음 잠금 해제)')


def main():
    restart_sound_player()
    kill_frontier_explorer()

    rclpy.init(args=['--ros-args', '-r', '/tf:=/j100_0915/tf',
                     '-r', '/tf_static:=/j100_0915/tf_static'])
    node = Node('finish_mission')

    holder = {}
    node.create_subscription(TagPoseArray, '/tags_in_map',
                             lambda m: holder.setdefault('tags', m), 10)
    node.create_subscription(OccupancyGrid, '/map',
                             lambda m: holder.setdefault('map', m), 1)

    print('[2/6] 태그/맵 데이터 수신 대기...')
    t0 = time.time()
    while ('tags' not in holder or 'map' not in holder) and time.time() - t0 < 8:
        rclpy.spin_once(node, timeout_sec=0.2)

    tags_msg = holder.get('tags')
    mapdata = holder.get('map')
    if tags_msg is None or not tags_msg.tags:
        print('실패: /tags_in_map 에 태그가 없음')
        return
    if mapdata is None:
        print('실패: /map 수신 안 됨')
        return

    xs = [t.pose.pose.position.x for t in tags_msg.tags]
    ys = [t.pose.pose.position.y for t in tags_msg.tags]
    avg_x, avg_y = sum(xs) / len(xs), sum(ys) / len(ys)
    print(f'[2/6] 태그 {len(xs)}개 평균(밀집 중심): ({avg_x:.2f}, {avg_y:.2f})')

    planner = PathPlanner(mapdata, robot_radius=0.25)
    target_grid = world_to_grid(mapdata, avg_x, avg_y)
    snapped = planner.nearest_walkable(target_grid)
    if snapped is None:
        print('실패: 평균 지점 근처에 정차 가능한 곳이 없음')
        return
    sx, sy = grid_to_world(mapdata, snapped).x, grid_to_world(mapdata, snapped).y
    print(f'[3/6] 정차 가능한 가장 가까운 지점으로 스냅: ({sx:.2f}, {sy:.2f})')

    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    pose = None
    t0 = time.time()
    while pose is None and time.time() - t0 < 6:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tfm = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = (tfm.transform.translation.x, tfm.transform.translation.y)
        except Exception:
            pass
    if pose is None:
        print('실패: 로봇 위치(TF)를 못 가져옴')
        return
    print(f'      현재 로봇 위치: ({pose[0]:.2f}, {pose[1]:.2f})')

    path, cost, reached = planner.plan(pose, (sx, sy), truncate_end_cells=0)
    if path is None:
        print('실패: A* 경로 계획 실패 — 갈 수 있는 길이 없음')
        return
    print(f'[4/6] 경로 계획 성공: {len(path)} waypoints, '
          f'{"완전 도달 가능" if reached else "부분경로(끝까지는 못 봄)"}')

    plan_pub = node.create_publisher(Path, '/plan', 10)
    goal_pub = node.create_publisher(Bool, '/final_goal_reached', 10)
    reached_holder = {}
    node.create_subscription(Bool, '/goal_reached',
                             lambda m: reached_holder.setdefault('r', m.data), 10)
    time.sleep(1.0)  # 구독자 디스커버리 대기

    msg = Path()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    for p in path:
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position = p
        ps.pose.orientation = Quaternion(w=1.0)
        msg.poses.append(ps)
    plan_pub.publish(msg)
    print('[4/6] 경로 발행 완료 — 이동 시작')

    print('[5/6] 도착 대기 중...')
    t0 = time.time()
    while 'r' not in reached_holder and time.time() - t0 < 180:
        rclpy.spin_once(node, timeout_sec=0.3)
    if 'r' in reached_holder:
        print('[5/6] 도착 확인됨 (/goal_reached)')
    else:
        print('[5/6] 180초 대기했지만 도착 신호 없음 — 그래도 완료 신호는 보냄')

    goal_pub.publish(Bool(data=True))
    time.sleep(0.5)
    goal_pub.publish(Bool(data=True))
    print('[6/6] /final_goal_reached 발행 → 완료음 재생됨(sound_player 가 살아있어야 함)')
    print('미션 종료.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
