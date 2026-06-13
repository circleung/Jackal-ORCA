#!/usr/bin/env bash
# DualSense 가 BT로 connected 인데 입력/ js0 장치가 안 생기는 문제 정리.
#   ① joydev 모듈 로드 (+부팅 자동)  — js0(레거시 조이스틱) 생성에 필요
#   ② ERTM 비활성 (+영구)            — PS 패드 BT HID 미생성 주범
#   ③ 패드 재연결 후 js0 확인
# 실행:  ! ~/colcon_ws/scripts/bt_dualsense_fix.sh   (sudo 비번 한 번 입력)
set -u
MAC=50:EE:32:F7:62:28

echo "── ① joydev 로드 + 부팅 자동 ──"
sudo modprobe joydev
echo joydev | sudo tee /etc/modules-load.d/joydev.conf >/dev/null
lsmod | grep -q joydev && echo "  joydev 로드됨" || echo "  ✗ joydev 로드 실패"

echo "── ② ERTM 비활성 (런타임 + 영구) ──"
echo 1 | sudo tee /sys/module/bluetooth/parameters/disable_ertm >/dev/null
echo 'options bluetooth disable_ertm=1' | sudo tee /etc/modprobe.d/bluetooth-ertm.conf >/dev/null
echo "  disable_ertm = $(cat /sys/module/bluetooth/parameters/disable_ertm)  (Y=꺼짐 정상)"

echo "── ③ 패드 재연결 ──"
bluetoothctl disconnect "$MAC" >/dev/null 2>&1; sleep 2
bluetoothctl connect "$MAC" >/dev/null 2>&1;    sleep 3

echo "── 결과 ──"
if [ -e /dev/input/js0 ]; then
  ls -l /dev/input/js0
  echo "✓ js0 생성! 이제 패드 L1(deadman) 누른 채 스틱으로 주행 테스트."
  echo "  안 먹으면 joy_node 가 옛 상태라 그럼 → joy_node 재시작 안내받기."
else
  echo "✗ 아직 js0 없음. 런타임 ERTM 변경이 기존 어댑터에 안 먹은 경우 →"
  echo "  확실한 방법: 'sudo reboot' 후 패드 켜기 (ERTM 영구설정 적용됨)."
  echo "  즉시 대안: USB-C 케이블로 패드 직결 (ERTM 무관, 바로 생성)."
fi
