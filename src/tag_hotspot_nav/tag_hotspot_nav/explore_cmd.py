"""explore_cmd.py — 터미널에서 탐사를 제어하는 단발 명령들.

워크스페이스 source 후 터미널에서 바로:
  go      : 이전 맵 리셋 + 탐사 처음부터 시작
  pause   : 일시정지 (로봇 즉시 정지, 상태 유지)
  resume  : 일시정지 해제 (blacklist 등 상태 그대로 이어감)
  save    : 현재 세션 저장 — 맵(.pgm/.yaml) + posegraph(.posegraph/.data)
            → ~/colcon_ws/src/tag_hotspot_nav/maps/map_<타임스탬프>.*

구현: go/pause/resume 은 /explore/command (std_msgs/String) 토픽 발행,
save 는 slam_toolbox 서비스 직접 호출.
"""

import datetime
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _send(cmd: str):
    rclpy.init()
    node = Node('explore_cmd_sender')
    pub = node.create_publisher(String, '/explore/command', 10)

    # DDS discovery: 구독자 수가 가변(frontier/pure_pursuit/sound/tag_centering/
    # map_cleaner/tag_collector …)이라 매직넘버 대신 count 안정화까지 대기.
    deadline = time.time() + 5.0
    last, stable = -1, 0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        cnt = pub.get_subscription_count()
        if cnt > 0 and cnt == last:
            stable += 1
            if stable >= 5:          # ~0.5s 동안 변화 없으면 매칭 완료로 간주
                break
        else:
            stable = 0
        last = cnt
    if pub.get_subscription_count() == 0:
        print('경고: 구독자 없음 — explore.launch.py 가 떠 있는지 확인하세요')

    msg = String(data=cmd)
    end = time.time() + 2.0          # 늦게 매칭되는 구독자 위해 2초간 반복 발행
    while time.time() < end:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    n_sub = pub.get_subscription_count()   # 반복발행 후 실제 매칭 수(디스커버리 완료 시점)
    node.destroy_node()
    rclpy.shutdown()
    print(f"탐사 명령 전송: '{cmd}' (구독자 {n_sub}개)")


def go():
    """탐사 시작/재개 (맵·상태 유지). 처음부터 다시 하려면 reset."""
    _send('go')


def pause():
    _send('pause')


def resume():
    _send('resume')


def reset():
    """매핑 처음부터: slam 맵 + 탐사 상태 + 태그 누적 + map_cleaner 잔재 전부 초기화."""
    _send('reset')


def _call_service(node, srv_type, name, request, timeout=15.0):
    client = node.create_client(srv_type, name)
    if not client.wait_for_service(timeout_sec=5.0):
        print(f'실패: {name} 서비스 없음 (slam_2d.launch.py 떠 있는지 확인)')
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result()


def save():
    """현재 SLAM 세션 저장: 점유격자맵 + 직렬화 posegraph."""
    from slam_toolbox.srv import SerializePoseGraph
    import subprocess
    import time

    out_dir = os.path.expanduser('~/colcon_ws/src/tag_hotspot_nav/maps')
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    base = os.path.join(out_dir, f'map_{stamp}')

    rclpy.init()
    node = Node('session_saver')

    # 점유격자맵 저장 헬퍼 — map_saver_cli (slam_toolbox SaveMap 서비스보다 안정적)
    def _save_grid(topic, fname):
        try:
            r = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-t', topic, '-f', fname,
                 '--ros-args', '-p', 'save_map_timeout:=8.0'],
                capture_output=True, timeout=30)
            return r.returncode == 0
        except Exception:
            return False

    # 1) /map → .pgm/.yaml (slam 이 /map 발행 중이면 안정적으로 저장됨)
    ok_map = _save_grid('/map', base)

    # 2) posegraph → 직렬화 (재개/재로드용)
    req2 = SerializePoseGraph.Request()
    req2.filename = base
    res2 = _call_service(node, SerializePoseGraph,
                         '/slam_toolbox/serialize_map', req2)
    ok_graph = res2 is not None and res2.result == SerializePoseGraph.Response.RESULT_SUCCESS

    # 3) 정리맵(/map_nav) — map_cleaner 가 돌 때만. 발행자 없으면 '실패' 아니라 '스킵'.
    has_mapnav = False
    deadline = time.time() + 1.5
    while time.time() < deadline:
        if node.count_publishers('/map_nav') > 0:
            has_mapnav = True
            break
        time.sleep(0.1)
    ok_clean = _save_grid('/map_nav', base + '_clean') if has_mapnav else None

    node.destroy_node()
    rclpy.shutdown()

    print(f"맵 저장: {'OK' if ok_map else '실패'} → {base}.pgm/.yaml")
    print(f"posegraph 저장: {'OK' if ok_graph else '실패'} → {base}.posegraph/.data")
    if has_mapnav:
        print(f"정리맵(/map_nav) 저장: {'OK' if ok_clean else '실패'} → {base}_clean.pgm/.yaml")
    else:
        print("정리맵(/map_nav): 스킵 (map_cleaner 미사용)")
