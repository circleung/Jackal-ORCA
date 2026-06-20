#!/bin/bash
# start_explore.sh — 탐사 스택 일괄 기동 (slam_2d → perception → explore)
#   · 플랫폼(roas2_bringup)은 건드리지 않음. 먼저 떠 있어야 함.
#   · reset/go 는 기동 완료 후 별도로 입력.
#
# 사용:
#   start_explore             # 탐사 스택 기동
set -u
WS="$HOME/colcon_ws"
EXPLORE_ARGS=""

set +u   # ROS setup.bash 가 미정의 변수(AMENT_TRACE_SETUP_FILES 등)를 참조하므로 잠시 해제
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

# 0) 플랫폼 확인 (없으면 중단 — 절대 우리가 안 띄움)
if ! ros2 node list 2>/dev/null | grep -q "/j100_0915/"; then
  echo "✗ Clearpath 플랫폼이 안 보입니다. 먼저 플랫폼을 띄우세요:"
  echo "    ros2 launch roas2_bringup platform.launch.py"
  exit 1
fi
echo "✓ 플랫폼 감지됨"

# 1) 옛 우리 스택/좀비 livox 정리 (플랫폼 패턴은 제외 — roas2/platform 안 건드림)
echo "── 옛 스택/좀비 정리 ──"
for pat in "slam_2d.launch.py" "perception.launch.py" "explore.launch.py" \
           "livox_ros_driver2_node" "async_slam_toolbox_node" \
           "pointcloud_to_laserscan_node" "frontier_explorer" "pure_pursuit"; do
  pids=$(pgrep -f "$pat")
  if [ -n "$pids" ]; then echo "  kill $pat ($pids)"; kill $pids 2>/dev/null; fi
done
sleep 3

# 2) SLAM
echo "── [1/3] slam_2d 기동 ──"
nohup ros2 launch tag_hotspot_nav slam_2d.launch.py > /tmp/slam_2d.log 2>&1 &
echo -n "  /scan 대기 "
ok=0
for i in $(seq 1 30); do
  if timeout 3 ros2 topic echo /scan --once >/dev/null 2>&1; then ok=1; break; fi
  echo -n "."; sleep 2
done
echo ""
if [ "$ok" != 1 ]; then
  echo "  ✗ /scan 이 60s 내 안 옵니다 → /tmp/slam_2d.log 확인 (라이다 연결/좀비 livox?)"
  exit 1
fi
echo "  ✓ /scan 수신 OK"

# 2b) slam_toolbox lifecycle 활성 확인 (가끔 DDS 노이즈로 activate 이벤트를 놓쳐
#     inactive 에 멈춤 → /map 안 나오고 탐사 안 됨). 강제 activate 로 보강.
echo -n "  slam_toolbox 활성 확인 "
mapok=0
for i in $(seq 1 10); do
  st=$(timeout 5 ros2 lifecycle get /slam_toolbox 2>/dev/null | grep -oE 'active|inactive|unconfigured|finalized' | head -1)
  if [ "$st" = "active" ]; then mapok=1; break; fi
  if [ "$st" = "inactive" ]; then
    echo -n "[activate 시도]"; timeout 10 ros2 lifecycle set /slam_toolbox activate >/dev/null 2>&1
  fi
  echo -n "."; sleep 2
done
echo ""
if [ "$mapok" = 1 ]; then echo "  ✓ slam_toolbox active → /map 발행"; else
  echo "  ⚠ slam_toolbox active 확인 실패 — 수동: ros2 lifecycle set /slam_toolbox activate"
fi

# 3) 인식 + 탐사
echo "── [2/3] perception 기동 ──"
nohup ros2 launch tag_hotspot_nav perception.launch.py > /tmp/perception.log 2>&1 &
echo "── [3/3] explore 기동  $EXPLORE_ARGS ──"
nohup ros2 launch tag_hotspot_nav explore.launch.py $EXPLORE_ARGS > /tmp/explore.log 2>&1 &

# 4) 노드 등록 확인
echo -n "  노드 등록·DDS 디스커버리 대기 "
sleep 13; echo ""
echo "── 기동된 노드 ──"
ros2 node list 2>/dev/null | grep -E \
  "slam_toolbox|frontier_explorer|pure_pursuit|tag_collector|map_cleaner|sound_player|stuck_detector" | sort

cat <<'EOF'

✅ 기동 완료. 이제 이 터미널(또는 source 된 새 터미널)에서:
   reset    # 처음부터 매핑 (맵·상태·태그·keep-out 전부 초기화)
   go       # 시작/재개
   pause    # 정지
   save     # 맵 + 정리맵 저장
로그:  tail -f /tmp/slam_2d.log  /tmp/explore.log
정지:  stop_explore   (플랫폼은 유지됨)
EOF
