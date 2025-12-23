# JetRacer

NVIDIA JetRacer AI Kit를 위한 Python 제어 라이브러리 및 유틸리티 모음입니다.
이 프로젝트는 **Multiplexer(Mux)** 구조를 기반으로 조이스틱과 UDP 제어를 통합 관리하며, `runner.py`를 통해 모든 프로세스를 한 번에 실행합니다.

## 디렉토리 구조

```
.
├── config/              # 차량 설정 (nvidia_racecar_config.json) 및 udev 규칙
├── jetracer/
│   ├── core/            # NvidiaRacecar 등 하드웨어 제어 클래스
│   ├── mux/             # (핵심) Mux, Joystick, UDP 수신 모듈
│   ├── runner.py        # (실행) 통합 런처 스크립트
│   └── diagnostics/     # 배터리 모니터링 도구
└── README.md
```

## 사전 준비

### 1. 환경 설정 (environment.yml)

Conda 환경을 생성하고 활성화합니다.

```bash
conda env create -f environment.yml
conda activate jet
```

### 2. 조이스틱 권한 설정 (1회만 수행)

조이스틱을 `sudo` 없이 사용하기 위해 udev 규칙을 설정합니다.

```bash
sudo cp config/99-joystick.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
설정 후 조이스틱을 다시 연결하세요.

### 3. 배터리 모니터 서비스 설정 (선택사항)

부팅 시 `battery_monitor`를 자동 실행하여 OLED에 IP와 배터리 상태를 표시합니다.

```bash
# 서비스 파일 생성 (사용자명에 맞게 자동 치환됨)
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

# 서비스 등록 및 시작 (username을 본인 계정명으로 변경)
sudo systemctl daemon-reload
sudo systemctl enable battery_monitor@<username>.service
sudo systemctl start  battery_monitor@<username>.service
```

## 사용 방법 (Unified Runner)

모든 제어 시스템(`mux`, `joystick`, `udp_recv`)을 통합 실행합니다.

```bash
python3 -m jetracer.runner --speed5-throttle 0.20
```

-   **--speed5-throttle**: UDP로 `speed=5.0` 명령을 받았을 때 적용할 스로틀 값 (기본: 0.20)
-   **--device**: 조이스틱 경로 (미지정 시 자동 감지)

### 동작 모드
`runner.py` 실행 중 조이스틱의 **'Y' 버튼**을 눌러 모드를 전환할 수 있습니다.
1.  **Joystick Mode**: 게임패드로 직접 제어
2.  **UDP Mode**: 외부 UDP 명령으로 제어 (AI 자율주행 등)

터미널 로그는 현재 활성화된 모드의 정보만 필터링되어 표시됩니다.

