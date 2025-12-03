
## 디렉토리 구조

```
jetracer/
├── core/                # Racecar, NvidiaRacecar 등 기본 차량 제어
├── teleop/              # 조이스틱/키보드/UDP 등 원격 조작 스크립트
└── diagnostics/         # 배터리 모니터 및 저속 보정 도구
    └── low_speed/       # 저속 매핑 실험 스크립트
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


## JetRacer Battery Monitor Auto Start (systemd)

1. systemd 템플릿 서비스 생성
sudo nano /etc/systemd/system/battery_monitor@.service

내용:

[Unit]
Description=JetRacer Battery Monitor (%i)
After=network-online.target

[Service]
Type=simple
User=%i
ExecStart=/usr/bin/python3 /home/%i/jetracer/jetracer/diagnostics/battery_monitor.py
Restart=always
RestartSec=2
WorkingDirectory=/home/%i/jetracer/jetracer/diagnostics

[Install]
WantedBy=multi-user.target

2. 활성화 및 실행
sudo systemctl daemon-reload
sudo systemctl enable battery_monitor@<username>.service
sudo systemctl start  battery_monitor@<username>.service

3. 상태/로그 확인
systemctl status battery_monitor@<username>.service
journalctl -u battery_monitor@<username>.service -f

주의

<username>에는 실제 로그인 계정명(whoami)을 넣는다.