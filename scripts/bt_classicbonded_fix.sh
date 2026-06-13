#!/usr/bin/env bash
# DualSense BT HID 거부("hidp_add_connection() Rejected connection from !bonded device") 해결.
# 원인: BlueZ input 플러그인 ClassicBondedOnly=true 가 패드의 HID 연결을 거부.
# 해결: ClassicBondedOnly=false 로 바꾸고 bluetooth 재시작.
#
# ★ 진짜 터미널에서 실행할 것 (sudo 비번 입력 필요):
#     ~/colcon_ws/scripts/bt_classicbonded_fix.sh
set -u
CONF=/etc/bluetooth/input.conf

echo "── ① input.conf: ClassicBondedOnly=false ──"
sudo sed -i -E '/^[[:space:]]*#?[[:space:]]*ClassicBondedOnly[[:space:]]*=/d' "$CONF"
sudo sed -i -E '/^\[General\]/a ClassicBondedOnly=false' "$CONF"
echo "  설정값: $(grep -iE 'ClassicBondedOnly' "$CONF")"

echo "── ② uhid/hid_playstation 로드(+영구) ──"
sudo modprobe uhid hid_playstation 2>/dev/null
printf 'uhid\nhid_playstation\njoydev\n' | sudo tee /etc/modules-load.d/dualsense.conf >/dev/null

echo "── ③ bluetooth 재시작 ──"
sudo systemctl restart bluetooth
sleep 3
systemctl is-active bluetooth

echo ""
echo "✓ 준비 끝. 이제 패드를 ★페어링 모드★(PS + Create 동시 ~5s, 라이트바 두 번씩 깜빡)로 하고:"
echo "    ~/colcon_ws/scripts/bt_dualsense_connect.sh"
echo "  연결 후 'ls -l /dev/input/js0' 로 확인."
