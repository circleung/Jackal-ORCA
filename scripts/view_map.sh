#!/bin/bash
# view_map.sh — 저장된 맵(.pgm/.yaml)을 /map_saved 토픽으로 발행해 Foxglove/RViz 에서 보기.
#   실행 중인 slam 의 /map 과 충돌하지 않도록 별도 토픽(/map_saved) 사용.
#
# 사용:
#   view_map                       # 가장 최근 맵
#   view_map map_20260610_013741   # 특정 맵 (이름 일부/전체/경로 OK)
#   view_map --list                # 저장된 맵 목록만
# 종료: Ctrl+C
set -u
MAPDIR="$HOME/colcon_ws/src/tag_hotspot_nav/maps"
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/colcon_ws/install/setup.bash"
set -u

if [ "${1:-}" = "--list" ]; then
  echo "저장된 맵 (최신순):"
  ls -t "$MAPDIR"/*.yaml 2>/dev/null | sed 's#.*/##;s#\.yaml##'
  exit 0
fi

# 1) yaml 파일 결정
arg="${1:-}"
if [ -z "$arg" ]; then
  yaml=$(ls -t "$MAPDIR"/*.yaml 2>/dev/null | head -1)
elif [ -f "$arg" ]; then
  yaml="$arg"
elif [ -f "$MAPDIR/${arg%.yaml}.yaml" ]; then
  yaml="$MAPDIR/${arg%.yaml}.yaml"
else
  yaml=$(ls -t "$MAPDIR"/*"$arg"*.yaml 2>/dev/null | head -1)
fi
if [ -z "${yaml:-}" ] || [ ! -f "$yaml" ]; then
  echo "✗ 맵 yaml 못 찾음: '${arg}'. 사용 가능:"
  ls -t "$MAPDIR"/*.yaml 2>/dev/null | sed 's#.*/##;s#\.yaml##'
  exit 1
fi
echo "맵: $(basename "$yaml")  →  토픽 /map_saved"

# 2) map_server 기동 (/map → /map_saved 리매핑)
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="$yaml" -p use_sim_time:=false \
  -r /map:=/map_saved -r /map_updates:=/map_saved_updates >/tmp/view_map.log 2>&1 &
MS_PID=$!
trap "kill $MS_PID 2>/dev/null; echo; echo '종료 — /map_saved 내림'; exit 0" INT TERM

# 3) lifecycle configure → activate (DDS 지연/타임아웃 대비 재시도 루프)
sleep 2
active=0
for i in $(seq 1 12); do
  st=$(timeout 5 ros2 lifecycle get /map_server 2>/dev/null | grep -oE 'active|inactive|unconfigured|finalized' | head -1)
  if [ "$st" = active ]; then active=1; break; fi
  if [ "$st" = inactive ]; then
    timeout 8 ros2 lifecycle set /map_server activate  >/dev/null 2>&1
  else
    timeout 8 ros2 lifecycle set /map_server configure >/dev/null 2>&1
  fi
  sleep 1.5
done
if [ "$active" = 1 ]; then echo "✓ /map_saved 발행 중."
else echo "⚠ map_server 활성 실패 — /tmp/view_map.log 확인"; fi
echo "  Foxglove: ws://<로봇IP>:8766 (탐사스택) 또는 :8765 (플랫폼) → Map 패널에 /map_saved"
echo "  RViz:     Map display, Topic=/map_saved (Durability=Transient Local)"
echo "  종료: Ctrl+C"

# 4) 종료까지 유지
wait $MS_PID
