# JetRacer Setup Guide

이 문서는 JetRacer를 처음 구동하기 위한 환경 설정 및 드라이버 설치 방법을 설명합니다.

## 1. 하드웨어 준비
- JetRacer AI Kit 조립 완료
- 조이스틱 (Logitech F710 또는 호환 기기)
- 충전된 배터리 연결

## 2. 환경 설정 (Conda)

환경 구축을 위해 Conda(Miniforge3 추천)를 사용합니다.

```bash
# 환경 생성 및 패키지 설치
conda env create -f environment.yml

# 환경 활성화
conda activate jet
```

## 3. 조이스틱 유권한 설정

조이스틱을 `sudo` 없이 사용하기 위해 udev 규칙을 한 번 등록해야 합니다.

```bash
# 규칙 파일 복사
sudo cp config/99-joystick.rules /etc/udev/rules.d/

# 규칙 리로드
sudo udevadm control --reload-rules
sudo udevadm trigger
```
설정 후 조이스틱을 뺏다 다시 연결하세요.

## 4. 배터리 모니터 OLED 서비스 등록

부팅 시 자동으로 OLED에 IP 주소와 배터리 상태를 표시하도록 systemd 서비스를 등록합니다.

```bash
# 서비스 파일 생성 (사용자명에 맞게 자동 치환)
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

서비스가 정상 작동하면 OLED 화면이 켜지며 차량 정보가 표시됩니다. 만약 화면이 켜지지 않는다면 `sudo journalctl -u battery_monitor@<username>` 명령어로 로그를 확인하세요.
