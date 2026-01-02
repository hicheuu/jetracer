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

## 5. 와이파이 자동 연결 설정 (Headless 필수)

JetRacer가 부팅될 때 GUI 로그인 없이도 와이파이가 자동으로 연결되도록 설정합니다. 이 설정을 하지 않으면 매번 모니터를 연결하여 비밀번호를 쳐야 할 수도 있습니다.

아래 명령어를 복사하여 터미널에 붙여넣으세요. 이미 저장된 모든 와이파이 프로필을 '시스템 전체 연결'로 변경합니다.

```bash
nmcli -t -f NAME,TYPE connection show | grep ":802-11-wireless" | cut -d: -f1 | while read -r conn; do
    # 현재 권한 설정 확인
    perms=$(nmcli -g connection.permissions connection show "$conn")
    
    if [ -n "$perms" ]; then
        echo "[FIX] $conn → 구성원을 전체 사용자로 변경 및 비밀번호 파일 저장 활성화"
        # 1. 모든 사용자 접근 허용 (시스템 전체 연결)
        # 2. Keyring 대신 설정 파일에 직접 암호 저장 (로그인 전 연결 가능)
        sudo nmcli connection modify "$conn" \
            connection.permissions "" \
            802-11-wireless-security.psk-flags 0
    else
        echo "[OK ] $conn 이미 시스템 전체 연결로 설정되어 있습니다."
    fi
done

# 변경사항 적용 (현재 연결된 와이파이 재시작)
# sudo nmcli connection up "<SSID_NAME>"
```
