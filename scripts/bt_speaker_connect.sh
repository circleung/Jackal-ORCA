#!/bin/bash
# bt_speaker_connect.sh — JBL Go 4 자동 연결 + 연결 유지 데몬
# systemd user 서비스(bt-speaker.service)가 부팅 시 실행.
# 스피커가 꺼져 있어도 켜지는 순간 15s 안에 자동 연결됨.
MAC="${BT_SPEAKER_MAC:-E8:26:CF:82:BE:88}"   # JBL Go 4
INTERVAL=15

while true; do
  if ! bluetoothctl info "$MAC" 2>/dev/null | grep -q "Connected: yes"; then
    bluetoothctl connect "$MAC" > /dev/null 2>&1 \
      && echo "$(date '+%H:%M:%S') connected $MAC"
  fi
  sleep "$INTERVAL"
done
