#!/bin/bash
# (구버전 호환 래퍼) jackal_ctl.sh go 로 대체됨 — claude.md §0.13 참조.
# 구버전의 버그: global_planner / mine_goal_sender 를 죽이기만 하고 재시작 안 함.
exec "$(dirname "$0")/jackal_ctl.sh" go "$@"
