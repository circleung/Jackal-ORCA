#!/usr/bin/env bash
# DualSense(PS5) 패드 재페어링/연결 헬퍼.
#
# 사용 전: 패드를 ★페어링 모드★로 진입시킬 것
#   PS 버튼 + Create(왼쪽 위 작은 버튼) 동시에 길게(~5s) →
#   라이트바가 "빠르게 두 번씩" 깜빡이면 페어링 모드.
#
# 그다음:  ~/colcon_ws/scripts/bt_dualsense_connect.sh
set -u
MAC=50:EE:32:F7:62:28

echo "── 어댑터 준비 ──"
bluetoothctl power on        >/dev/null
bluetoothctl pairable on     >/dev/null
bluetoothctl agent on        >/dev/null
bluetoothctl default-agent   >/dev/null 2>&1

echo "── 스캔 20s (패드가 페어링 모드여야 함) ──"
bluetoothctl --timeout 20 scan on >/dev/null

if ! bluetoothctl devices | grep -qi "$MAC"; then
  echo "✗ DualSense($MAC) 미발견 — 페어링 모드 다시 확인(PS+Create 길게)하고 재실행"
  exit 1
fi

echo "── pair → trust → connect ──"
bluetoothctl pair    "$MAC"
bluetoothctl trust   "$MAC"
bluetoothctl connect "$MAC"

sleep 2
echo ""
echo "── 결과 ──"
bluetoothctl info "$MAC" | grep -iE "Paired|Trusted|Connected"
if [ -e /dev/input/js0 ]; then
  echo "✓ /dev/input/js0 생성됨 — joy_node 가 읽을 수 있음"
  echo "  (joy_node 가 시작 시 js0 를 못 잡았으면 한 번 재시작 필요)"
else
  echo "✗ /dev/input/js0 없음 — 연결은 됐는데 joydev 미생성. dmesg 확인 필요"
fi
