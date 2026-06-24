# 새 머신 셋업 가이드 (SETUP.md)

> 프로젝트를 깃에서 클론한 뒤 **재구축**할 때 필요한 운영 환경 설정.
> `~/.bashrc`는 git에 포함되지 않으므로, 아래 내용을 새 머신의 `~/.bashrc` 끝에 추가해야
> `start_explore` 등 단축 명령과 ROS 환경이 동작한다.

---

## 1. 워크스페이스 복원 & 빌드

```bash
# 1) 클론
cd ~ && git clone <이 저장소 URL> colcon_ws && cd colcon_ws

# 2) 벤더 패키지 복원 (.gitignore로 제외된 것들)
vcs import src < dependencies.repos        # clearpath_*, livox_ros_driver2, micro_ros, uros 등

# 3) Livox SDK (system install — driver 빌드 의존)
git clone https://github.com/Livox-SDK/Livox-SDK2.git ~/Livox-SDK2
cd ~/Livox-SDK2 && mkdir -p build && cd build && cmake .. && make -j && sudo make install
cd ~/colcon_ws

# 4) 빌드 (⚠ --symlink-install 쓰지 말 것 — entry point 깨짐)
colcon build
source install/setup.bash
```

## 2. `~/.bashrc` 에 추가할 운영 라인

```bash
# FastDDS 공유메모리(SHM) 전송 끔 — /dev/shm 잔재로 인한
# "RTPS_TRANSPORT_SHM ... open_and_lock_file failed" 노이즈 제거 (로컬은 UDP 사용)
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
export PATH="$HOME/.local/bin:$PATH"

alias rviz='ros2 launch roas2_bringup view_robot.launch.py namespace:=j100_0915 use_sim_time:=false'

# tag_hotspot_nav 탐사 스택 단축 명령
alias start_explore='bash ~/colcon_ws/scripts/start_explore.sh'
alias stop_explore='bash ~/colcon_ws/scripts/stop_explore.sh'
alias view_map='bash ~/colcon_ws/scripts/view_map.sh'
alias view_tags='python3 ~/colcon_ws/scripts/view_tags.py'
```

> `go`/`pause`/`resume`/`save`/`reset`은 `tag_hotspot_nav` 패키지 실행파일이라
> `source install/setup.bash` 하면 PATH(`install/tag_hotspot_nav/bin`)에 자동 포함된다.

## 3. 실행 순서

1. 플랫폼 기동(수동): `ros2 launch roas2_bringup platform.launch.py`
2. 탐사 스택: `start_explore` (slam_2d → perception → explore → 종료 리스너 자동 기동)
3. 매핑 시작/정지: `reset`(처음부터) / `go`(시작·재개) / `pause` / `save`
4. 수동 종료(태그 밀집지점 이동): `ros2 topic pub --times 5 /finish_exploration std_msgs/msg/Bool 'data: true'`

자세한 미션 흐름은 `start.md`, 아키텍처는 `src/tag_hotspot_nav/ARCHITECTURE.md` 참고.
