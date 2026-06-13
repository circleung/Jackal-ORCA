#!/bin/bash
# bt_dualsense_daemon.sh — DualSense(PS5) 패드 자동 연결 + 연결 유지 데몬
# systemd user 서비스(bt-dualsense.service)가 부팅 시 실행.
# 패드가 꺼져 있어도 켜는 순간(PS 버튼) 15s 안에 자동 연결 시도.
#
# 전제(메모리 2026-06-08 기록):
#   · /etc/bluetooth/input.conf 의 ClassicBondedOnly=false 적용돼 있어야
#     DualSense HID 연결이 거부되지 않음 (이미 적용됨)
#   · joydev/uhid 모듈 로드 + ERTM 비활성 (이미 영구 설정됨)
#   · 최초 1회는 ★페어링 모드★로 bond 수립 필요(bt_dualsense_connect.sh).
#     이후 이 데몬이 연결을 유지/복구함.
MAC="${BT_DUALSENSE_MAC:-50:EE:32:F7:62:28}"   # DualSense Wireless Controller
INTERVAL=15

# 어댑터/에이전트 준비 (한 번)
bluetoothctl power on      >/dev/null 2>&1
bluetoothctl agent on      >/dev/null 2>&1
bluetoothctl default-agent >/dev/null 2>&1

while true; do
  if ! bluetoothctl info "$MAC" 2>/dev/null | grep -q "Connected: yes"; then
    bluetoothctl trust "$MAC"   >/dev/null 2>&1
    bluetoothctl connect "$MAC" >/dev/null 2>&1 \
      && echo "$(date '+%H:%M:%S') connected $MAC"
  fi
  sleep "$INTERVAL"
done
