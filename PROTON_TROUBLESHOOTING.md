# Proton ROS2 CRC16/Invalid Header Error 해결 가이드

## 에러 설명
- **CRC16 Error**: 시리얼 통신 중 데이터 패킷의 체크섬이 일치하지 않음
- **Invalid Header Error**: 패킷 헤더 (0x50, 0x52)가 올바르지 않음

이 에러들은 MCU와 컴퓨터 간 시리얼 통신 문제를 나타냅니다.

## 주요 원인

### 1. 중복 프로세스 실행
**증상**: 이미 clearpath-platform 서비스가 실행 중인데 수동으로 또 실행
**확인 방법**:
```bash
# 서비스 상태 확인
systemctl status clearpath-platform

# 실행 중인 proton 프로세스 확인
ps aux | grep proton_ros2_node

# 시리얼 포트 사용 확인
fuser /dev/ttyACM0
```

**해결 방법**:
```bash
# 서비스를 사용하는 경우
sudo systemctl stop clearpath-platform
sudo systemctl start clearpath-platform

# 수동 실행을 사용하는 경우
sudo systemctl stop clearpath-platform
cd /usr/sbin
sudo ./clearpath-platform-start
```

### 2. 시리얼 포트 설정 문제
**확인 방법**:
```bash
# 시리얼 포트 확인
ls -la /dev/clearpath/j100

# 시리얼 포트 설정 확인
stty -F /dev/ttyACM0 -a
```

**참고**: Proton 프로토콜은 자동으로 적절한 baud rate를 설정합니다.

### 3. MCU 펌웨어 버전 불일치
**확인 방법**:
```bash
# 펌웨어 버전 확인
ros2 topic echo /j100_0915/platform/mcu/status --once | grep firmware_version
```

**참고**: Proton 프로토콜은 최소 펌웨어 버전 3.0.0이 필요합니다.

### 4. 시작 시 일시적 동기화 에러
**증상**: 시작 직후 몇 개의 에러 메시지가 나타나지만 곧 정상화됨
**원인**: MCU와 컴퓨터가 통신 프로토콜을 동기화하는 과정

이 경우는 정상이며, 에러가 지속되지 않으면 문제가 아닙니다.

## 현재 시스템 상태 확인 방법

### MCU 통신 상태 확인
```bash
# IMU 데이터 확인 (정상이면 ~40Hz)
timeout 3 ros2 topic hz /j100_0915/sensors/imu_0/data_raw

# MCU 상태 확인
ros2 topic echo /j100_0915/platform/mcu/status --once

# 모터 피드백 확인
ros2 topic echo /j100_0915/platform/motors/feedback --once
```

### 실시간 로그 확인
```bash
# Proton 노드 로그 찾기
ls -lt /tmp/proton_ros2_node*.log | head -1

# 최신 로그 실시간 모니터링
tail -f $(ls -t /tmp/proton_ros2_node*.log | head -1)
```

### Diagnostics 확인
```bash
# 전체 진단 정보
ros2 topic echo /j100_0915/diagnostics

# Proton 관련 진단만
ros2 topic echo /j100_0915/diagnostics | grep -A 10 "proton"
```

## 문제 해결 단계

### 1단계: 현재 상태 확인
```bash
# 데이터가 정상적으로 오는지 확인
timeout 2 ros2 topic hz /j100_0915/sensors/imu_0/data_raw
```
- **데이터가 오는 경우**: 정상 작동 중. 초기 에러는 무시해도 됨
- **데이터가 없는 경우**: 2단계로 이동

### 2단계: 프로세스 재시작
```bash
# 실행 중인 프로세스 종료
sudo pkill -f clearpath-platform-start
sudo pkill -f proton_ros2_node

# 잠시 대기
sleep 2

# 서비스 재시작
sudo systemctl restart clearpath-platform

# 또는 수동 실행
cd /usr/sbin
sudo ./clearpath-platform-start
```

### 3단계: 하드웨어 확인
```bash
# USB 연결 확인
lsusb | grep STMicroelectronics

# 시리얼 포트 확인
ls -la /dev/ttyACM*

# /dev/clearpath/j100 링크 확인
ls -la /dev/clearpath/
```

### 4단계: 상세 진단
현재 시스템의 로그 파일을 확인하고 문제를 분석합니다:
```bash
# 최근 로그에서 에러 검색
grep -i "error\|failed\|crc\|header" $(ls -t /tmp/proton_ros2_node*.log | head -1) | tail -50

# systemd 서비스 로그 확인
journalctl -u clearpath-platform -n 100 --no-pager
```

## 참고 사항

- 시작 시 1-2개의 CRC/Header 에러는 정상일 수 있습니다 (동기화 과정)
- 에러가 지속적으로 발생하면 케이블이나 MCU 하드웨어 점검이 필요합니다
- /dev/ttyACM0이 /dev/clearpath/j100로 올바르게 링크되어 있어야 합니다

## 현재 시스템 정보

생성 시각: 2026년 4월 3일
- MCU 디바이스: /dev/clearpath/j100 -> /dev/ttyACM0
- USB 디바이스: STMicroelectronics Virtual COM Port
- 플랫폼: J100
- 네임스페이스: j100_0915
- Proton 패키지 버전: 1.0.0
