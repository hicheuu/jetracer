# JetRacer

NVIDIA JetRacer AI Kit를 위한 Python 제어 라이브러리 및 유틸리티 모음입니다.

## 사전 준비

### 1. 환경 설정 (environment.yml)

프로젝트를 사용하기 전에 Conda 환경을 설정해야 합니다.

```bash
# Conda 환경 생성 및 활성화
conda env create -f environment.yml
conda activate jet
```

**주요 의존성:**
- Python 3.8
- Adafruit CircuitPython 라이브러리 (PCA9685, INA219, SSD1306 등)
- Jetson.GPIO (Jetson 보드용 GPIO 제어)
- 기타 하드웨어 제어 라이브러리

자세한 패키지 목록은 `environment.yml` 파일을 참조하세요.

### 2. 배터리 모니터 자동 시작 설정 (선택사항)

JetRacer의 `battery_monitor`를 시스템 부팅 시 자동으로 실행하도록 설정할 수 있습니다.

#### 서비스 템플릿 파일 생성

터미널에서 아래 명령으로 템플릿 유닛 파일을 생성하세요 (`sudo` 권한 필요).

```sh
sudo tee /etc/systemd/system/battery_monitor@.service > /dev/null << 'EOF'
[Unit]
Description=JetRacer Battery Monitor (%i)
After=network-online.target

[Service]
Type=simple
User=%i
WorkingDirectory=/home/%i/jetracer
ExecStart=/home/%i/miniforge3/envs/jet/bin/python3 -m jetracer.diagnostics.battery_monitor
Restart=always
RestartSec=2
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
EOF
```

**설명:**
- `ExecStart`는 모듈 방식(`-m jetracer.diagnostics.battery_monitor`)으로 실행하도록 권장합니다. 이렇게 하면 경로 문제를 줄일 수 있습니다.
- 가상환경을 사용 중이면 `ExecStart`에 가상환경의 `python` 절대 경로를 지정하세요 (예: `/home/<user>/venv/bin/python -m jetracer.diagnostics.battery_monitor`).

#### 데몬 재로딩 및 서비스 활성화/시작

```sh
sudo systemctl daemon-reload
sudo systemctl enable battery_monitor@<username>.service
sudo systemctl start  battery_monitor@<username>.service
```

#### 상태 및 로그 확인

```sh
sudo systemctl status battery_monitor@<username>.service
sudo journalctl -u battery_monitor@<username>.service -f
```

**주의사항:**
- `<username>`을 실제 시스템 사용자명(예: `pi`, `ubuntu`, 또는 `youruser`)으로 바꾸세요.
- I2C 장치(INA219, SSD1306)를 사용하는 경우 I2C 인터페이스가 활성화되어 있고 서비스 사용자가 장치에 접근할 권한이 있어야 합니다. 필요하면 `i2c` 그룹에 사용자를 추가하세요.
- 가상환경을 사용할 때는 `ExecStart`를 해당 가상환경의 python으로 변경하세요.

## 디렉토리 구조

```
jetracer/
├── core/                # Racecar, NvidiaRacecar 등 기본 차량 제어 클래스
├── teleop/              # 조이스틱/키보드/UDP 등 원격 조작 스크립트
├── sensors/             # IMU 등 센서 데이터 읽기 및 처리
└── diagnostics/         # 배터리 모니터 및 저속 보정 도구
    └── low_speed/       # 저속 매핑 실험 스크립트
```

```
9DOF_Razor_IMU/          # SparkFun 9DoF Razor IMU M0 Firmware
└── Firmware/            # Firmware만 담당 (기능 없음)
```

- 기존 `from jetracer.nvidia_racecar import NvidiaRacecar` 경로는 그대로 동작하지만, 새 구조를 직접 사용할 경우 `from jetracer.core import NvidiaRacecar`를 권장합니다.
- 각 스크립트는 독립 실행이 가능하며, 실행 시점에 JetRacer 하드웨어가 연결되어 있어야 합니다.

## 주요 스크립트

- `jetracer/teleop/joystick.py` : 게임패드/조이스틱으로 조종
- `jetracer/teleop/keyboard_drive.py` : 키보드 W/A/S/D 기반 스로틀·조향 제어
- `jetracer/teleop/udp_recv_drive.py` : UDP 명령 수신 후 차량 제어
- `jetracer/diagnostics/battery_monitor.py` : INA219 + SSD1306 기반 배터리·네트워크 상태 표시
- `jetracer/diagnostics/low_speed/interactive.py` : 저속 영역 인터랙티브 조정
- `jetracer/diagnostics/low_speed/pwm_sweep.py` : 일정 속도로 PWM 스윕 테스트

## 사용 예시

```bash
# 키보드 드라이브 (기본 스텝값 사용)
python -m jetracer.teleop.keyboard_drive

# 조이스틱 사용 (스케일/축 인덱스 지정)
python -m jetracer.teleop.joystick --steer-axis 2 --throttle-scale 0.4

# 배터리 모니터
python -m jetracer.diagnostics.battery_monitor
```

각 스크립트는 `Ctrl+C` 로 안전하게 종료되며, 종료 블록에서 스로틀과 스티어링을 0으로 리셋합니다.

## 각 디렉토리 상세 정보

각 디렉토리의 기능과 사용법은 해당 디렉토리의 README.md를 참조하세요:
- [core/README.md](jetracer/core/README.md) - 차량 제어 클래스
- [teleop/README.md](jetracer/teleop/README.md) - 원격 조작 스크립트
- [sensors/README.md](jetracer/sensors/README.md) - 센서 데이터 처리
- [diagnostics/README.md](jetracer/diagnostics/README.md) - 진단 및 보정 도구
