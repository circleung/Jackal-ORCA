# reference/

폐기한 `jackal_mine_detection` 패키지에서 참고용으로 보존한 코드.

- `mission_node.py` — 도킹 기록 방식 태그 접근 FSM
  (IDLE→SCANNING→APPROACHING_TAG→COOLDOWN, front:bearing→0 전진 / back:후진).
  tag_hotspot_nav 3단계(태그 접근) 구현 시 서보잉 로직 참고용.
  주의: 옛 스택(FAST-LIO /Odometry, 도킹 기록 방식) 기준 — 현재는 solvePnP 누적 방식이라
  접근 타깃 좌표 산출이 다름. 서보잉/FSM 골격만 참고.
