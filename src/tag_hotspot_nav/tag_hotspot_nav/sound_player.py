"""
sound_player.py — 이벤트 사운드 (mpg123, 출력 sink = JBL Go 4).

매핑 (2026-06-09 사용자 확정):
  맵핑 시작~유지        → scifi_beeps.mp3  (배경 루프, 탐사 중 / pause 시 정지)
  태그 포착(/tag_new)   → scanner.mp3      (원샷)
  장애물 앞            → lock_on.mp3      (/obstacle_block 상승엣지, 원샷)
  수동 전환 OR 끼임     → pullup_alarm.mp3 (조이패드 deadman 누름 1회 | /stuck 상승엣지)
  미션 완료(최종 도착)  → windows-xp-startup (마지막 hotspot 도착 = /final_goal_reached 상승엣지, 원샷)

입력: /explore/command(String), /tag_new(Int32), /obstacle_block(Bool),
      /stuck(Bool), joy_topic(Joy, deadman 버튼), /safety/state(String),
      /final_goal_reached(Bool)
"""
from __future__ import annotations
import os
import re
import shutil
import subprocess
import signal
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from sensor_msgs.msg import Joy

os.environ.setdefault('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')

_RE_PAUSED = re.compile(r'paused=(True|False)')


class SoundPlayerNode(Node):
    def __init__(self) -> None:
        super().__init__('sound_player')

        default_dir = os.path.expanduser('~/colcon_ws/sounds')
        self.declare_parameter('sounds_dir', default_dir)
        self.declare_parameter('player', 'mpg123')
        self.declare_parameter('enable_loop', True)
        self.declare_parameter('joy_topic', '/j100_0915/joy_teleop/joy')
        self.declare_parameter('deadman_button', 4)   # teleop_joy enable_button (L1)
        self.declare_parameter('danger_rearm', 3.0)   # [s] 위험 해제 이만큼 지속돼야 다시 울림
        # 사운드별 게인 (mpg123 -f scale, 32768=100%). 스캔음(beeps)은 배경이라 낮추고,
        # 태그 셔터음(찰칵)은 이벤트라 키운다. >1.0 은 증폭(짧은 원샷이라 약간의 클리핑 무방).
        self.declare_parameter('beeps_gain', 0.3)   # 맵핑 배경음(스캔) 볼륨 배율
        self.declare_parameter('tag_gain', 1.8)     # 태그 셔터음(찰칵) 볼륨 배율

        self._dir = os.path.expanduser(str(self.get_parameter('sounds_dir').value))
        self._player = str(self.get_parameter('player').value)
        self._loop_on = bool(self.get_parameter('enable_loop').value)
        self._deadman = int(self.get_parameter('deadman_button').value)

        self._snd = {
            'beeps':   os.path.join(self._dir, 'scifi_beeps.mp3'),
            'scanner': os.path.join(self._dir, 'scanner.mp3'),
            'tag':     os.path.join(self._dir, 'zvuk-fotoapparata.mp3'),  # 태그 포착 = 카메라 셔터음
            'lock_on': os.path.join(self._dir, 'lock_on.mp3'),
            'alarm':   os.path.join(self._dir, 'pullup_alarm.mp3'),
            'start':   os.path.join(self._dir, 'prowler-sound-effect_6bXErot.mp3'),
            'finish':  os.path.join(self._dir, 'windows-xp-startup_1ph012N.mp3'),  # 미션 완료(최종 도착)
        }
        self._gain = {
            'beeps': float(self.get_parameter('beeps_gain').value),
            'tag':   float(self.get_parameter('tag_gain').value),
        }

        self._exploring = False
        self._paused = False
        self._obstacle = False
        self._stuck = False
        self._deadman_prev = False
        self._danger_armed = True       # 위험(근접/계단) lock_on 울릴 준비
        self._clear_t = None            # 위험 해제 지속 시각
        self._rearm = float(self.get_parameter('danger_rearm').value)
        self._loop_proc: Optional[subprocess.Popen] = None
        self._announce_proc: Optional[subprocess.Popen] = None  # go/reset 시작음(prowler→scanner)
        self._last_announce = -1e9      # 반복발행/연타 디바운스
        self._final_done = False        # 미션완료(/final_goal_reached) 1회만 재생 래치

        self.create_subscription(String, '/explore/command', self._on_command, 10)
        self.create_subscription(Int32, '/tag_new', self._on_tag_new, 10)
        self.create_subscription(Bool, '/obstacle_block', self._on_obstacle, 10)
        self.create_subscription(Bool, '/stuck', self._on_stuck, 10)
        self.create_subscription(String, '/safety/state', self._on_safety, 10)
        self.create_subscription(Bool, '/final_goal_reached', self._on_final, 10)
        self.create_subscription(Joy, str(self.get_parameter('joy_topic').value),
                                 self._on_joy, 10)
        self.create_timer(0.5, self._loop_tick)

        missing = [k for k, v in self._snd.items() if not os.path.isfile(v)]
        if missing:
            self.get_logger().warn(f'사운드 파일 없음: {missing} (dir={self._dir})')
        if shutil.which(self._player) is None:
            self.get_logger().error(f'{self._player} 미설치 — 사운드 비활성')
        self.get_logger().info(
            f'sound_player up: 맵핑=beeps, 태그=셔터음, 장애물=lock_on, 수동/끼임=pullup')

    def _scale_args(self, key: str) -> list:
        """사운드별 게인 → mpg123 -f scale 인자 (32768=100%). 게인 1.0 이면 옵션 없음."""
        g = self._gain.get(key, 1.0)
        if g == 1.0:
            return []
        return ['-f', str(max(0, int(round(32768 * g))))]

    def _play(self, key: str) -> None:
        path = self._snd[key]
        if not os.path.isfile(path):
            return
        try:
            subprocess.Popen([self._player, '-q'] + self._scale_args(key) + [path],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().warn(f'재생 실패 {key}: {e}')

    def _announce(self) -> None:
        """go/reset 시작음: prowler 바로 → 이어서 scanner (mpg123 다중파일 순차재생)."""
        paths = [p for p in (self._snd['start'], self._snd['scanner']) if os.path.isfile(p)]
        if not paths:
            return
        try:
            if self._announce_proc is not None and self._announce_proc.poll() is None:
                self._announce_proc.send_signal(signal.SIGTERM)
            self._announce_proc = subprocess.Popen(
                [self._player, '-q'] + paths,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().warn(f'시작음 재생 실패: {e}')

    # ── 배경 루프 (맵핑 중 scifi_beeps) ──────────────────────────────
    def _loop_should_run(self) -> bool:
        announcing = self._announce_proc is not None and self._announce_proc.poll() is None
        return self._loop_on and self._exploring and not self._paused and not announcing

    def _loop_tick(self) -> None:
        running = self._loop_proc is not None and self._loop_proc.poll() is None
        want = self._loop_should_run()
        if want and not running:
            path = self._snd['beeps']
            if os.path.isfile(path):
                self._loop_proc = subprocess.Popen(
                    [self._player, '-q'] + self._scale_args('beeps') + ['--loop', '-1', path],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        elif not want and running:
            self._loop_proc.send_signal(signal.SIGTERM)
            self._loop_proc = None

    # ── 콜백 ─────────────────────────────────────────────────────────
    def _on_command(self, msg: String) -> None:
        c = msg.data.strip().lower()
        if c in ('go', 'resume', 'reset'):
            self._exploring = True       # → (시작음 끝난 뒤) 배경 beeps 루프 시작
            self._final_done = False     # 새 미션 → 완료음 래치 해제(다음 최종도착 때 다시 울림)
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_announce > 3.0:   # 2초 반복발행/연타를 1회로 수렴
                self._last_announce = now
                self._announce()         # prowler 바로 → 이어서 scanner
        elif c == 'pause':
            self._exploring = False

    def _on_tag_new(self, msg: Int32) -> None:
        self._play('tag')                # 태그 포착 = 카메라 셔터음(zvuk-fotoapparata.mp3)

    def _on_obstacle(self, msg: Bool) -> None:
        self._obstacle = bool(msg.data)
        self._eval_danger()

    def _eval_danger(self) -> None:
        # 근접물체 = 위험. 처음 생길 때 lock_on 1회, 깜빡임엔 재울림 안 함.
        # 위험이 danger_rearm 초 이상 깨끗이 사라진 뒤에야 다음 위험에 다시 울림.
        now = self.get_clock().now().nanoseconds * 1e-9
        danger = self._obstacle
        if danger:
            if self._danger_armed:
                self._play('lock_on')
                self._danger_armed = False
            self._clear_t = None
        else:
            if self._clear_t is None:
                self._clear_t = now
            elif not self._danger_armed and (now - self._clear_t) > self._rearm:
                self._danger_armed = True

    def _on_stuck(self, msg: Bool) -> None:
        if msg.data and not self._stuck:       # 상승엣지
            self._play('alarm')
        self._stuck = bool(msg.data)

    def _on_joy(self, msg: Joy) -> None:
        # deadman(enable) 버튼 누름 = 수동 전환 → 1회 pullup
        pressed = (len(msg.buttons) > self._deadman and
                   msg.buttons[self._deadman] == 1)
        if pressed and not self._deadman_prev:
            self._play('alarm')
        self._deadman_prev = pressed

    def _on_safety(self, msg: String) -> None:
        m = _RE_PAUSED.search(msg.data)
        if m:
            self._paused = (m.group(1) == 'True')

    def _on_final(self, msg: Bool) -> None:
        # 미션 완료 = 마지막 클러스터(hotspot) 최종 도착 → 배경음 끄고 windows-xp 시작음 1회.
        # (hotspot_navigator 가 모든 hotspot 접근 완료 후 /final_goal_reached True 발행)
        if not msg.data or self._final_done:
            return
        self._final_done = True
        self._exploring = False          # 배경 beeps 루프 정지 트리거(_loop_tick)
        if self._loop_proc is not None and self._loop_proc.poll() is None:
            self._loop_proc.send_signal(signal.SIGTERM)   # 틱(0.5s) 기다리지 말고 즉시 정지
            self._loop_proc = None
        self._play('finish')
        self.get_logger().info('🏁 미션 완료(/final_goal_reached) → windows-xp 시작음 재생')

    def destroy_node(self) -> None:
        if self._loop_proc is not None and self._loop_proc.poll() is None:
            self._loop_proc.send_signal(signal.SIGTERM)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SoundPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
