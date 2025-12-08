# teleop - 원격 조작 스크립트

다양한 입력 장치를 통해 JetRacer를 원격으로 제어하는 스크립트 모음입니다.

## 주요 스크립트

### `joystick.py`
게임패드/조이스틱으로 차량을 조종합니다.

**사용법:**
```bash
python -m jetracer.teleop.joystick
```

**옵션:**
- `--steer-axis`: 스티어링 축 인덱스 (기본값: 0)
- `--throttle-axis`: 스로틀 축 인덱스 (기본값: 4)
- `--invert-steer`: 스티어링 반전
- `--invert-throttle`: 스로틀 반전 (기본 활성화)
- `--deadzone`: 축 데드존 (기본값: 0.08)
- `--throttle-mode`: 스로틀 모드 (`stick` 또는 `trigger`, 기본값: `stick`)
- `--steer-scale`: 스티어링 스케일 (기본값: 1.0)
- `--throttle-scale`: 스로틀 스케일 (기본값: 0.125)

**예시:**
```bash
# 스로틀 스케일 조정 (최대 속도 제한)
python -m jetracer.teleop.joystick --throttle-scale 0.4

# 스티어링 축 변경
python -m jetracer.teleop.joystick --steer-axis 2
```

### `keyboard_drive.py`
키보드로 차량을 조종합니다 (W/A/S/D 키 사용).

**사용법:**
```bash
python -m jetracer.teleop.keyboard_drive
```

**조작법:**
- `W`: 전진 (스로틀 증가)
- `S`: 후진 (스로틀 감소)
- `A`: 왼쪽 회전 (스티어링 감소)
- `D`: 오른쪽 회전 (스티어링 증가)
- `Q`: 종료

### `udp_recv_drive.py`
UDP 패킷을 수신하여 차량을 제어합니다.

**사용법:**
```bash
python -m jetracer.teleop.udp_recv_drive
```

네트워크를 통해 원격으로 차량을 제어할 때 사용합니다.

### `keyboard_teleop.py`
키보드로 텔레옵 제어 (다양한 키 매핑 지원).

### `keyboard_yaw_hold.py`
키보드 제어 + IMU 기반 Yaw 보정.

### `ch_motor.py`
개별 모터 제어 스크립트.

## 공통 사항

- 모든 스크립트는 `Ctrl+C`로 안전하게 종료됩니다.
- 종료 시 스로틀과 스티어링이 자동으로 0으로 리셋됩니다.
- 실행 시 JetRacer 하드웨어가 연결되어 있어야 합니다.

