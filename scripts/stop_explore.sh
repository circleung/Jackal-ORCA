#!/bin/bash
# stop_explore.sh — 우리 탐사 스택만 정지. 플랫폼(roas2_bringup)은 절대 안 끔.
#   사용: stop_explore
set -u
set +u   # ROS setup.bash 가 미정의 변수를 참조하므로 잠시 해제
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$HOME/colcon_ws/install/setup.bash" 2>/dev/null
set -u

echo "── 탐사 스택 정지 (플랫폼 제외) ──"
for pat in "explore.launch.py" "perception.launch.py" "slam_2d.launch.py" \
           "frontier_explorer" "pure_pursuit" "tag_collector" "tag_centering" \
           "map_cleaner" "stuck_detector" "sound_player" \
           "async_slam_toolbox_node" "pointcloud_to_laserscan_node" \
           "livox_ros_driver2_node"; do
  pids=$(pgrep -f "$pat")
  if [ -n "$pids" ]; then echo "  kill $pat ($pids)"; kill $pids 2>/dev/null; fi
done
sleep 2
echo "✓ 정지 완료. 남은 비-플랫폼 노드:"
ros2 node list 2>/dev/null | grep -vE "/j100_0915/|/parameter_events|/rosout" || echo "  (없음 — 플랫폼만 남음)"
