#!/bin/bash
# finish_exploration_now.sh — 탐사 자동종료를 기다리지 않고 강제로 다음 단계(클러스터 계산
# → 태그 밀집지점 이동)로 넘기는 스크립트.
#
# 사용법 (Jackal에서):
#   ssh jackal
#   ~/finish_exploration_now.sh
#
# 동작:
#   1) frontier_explorer 일시정지 — 더 이상 새 탐사 경로를 발행하지 않게 함
#   2) /finish_exploration 에 true 발행 — hotspot_navigator(auto_start=True)가
#      자동으로 클러스터 계산 → 태그 밀집지점 이동 → 도달까지 진행

ROS_SETUP=". /opt/ros/jazzy/setup.bash && . /home/jackal/colcon_ws/install/setup.bash"

echo "[1/2] frontier_explorer 일시정지..."
bash -c "$ROS_SETUP && ros2 topic pub --once /explore/command std_msgs/msg/String 'data: pause'"

echo "[2/2] 탐사 완료 신호 발행 → hotspot_navigator 시작..."
bash -c "$ROS_SETUP && ros2 topic pub --once /finish_exploration std_msgs/msg/Bool 'data: true'"

echo "완료 — hotspot_navigator가 자동으로 클러스터 이동을 시작했을 것입니다."
echo "로그 확인: tail -f /tmp/explore.log"
