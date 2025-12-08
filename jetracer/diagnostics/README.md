# diagnostics - 진단 및 보정 도구

JetRacer의 배터리 모니터링 및 저속 보정 도구를 제공합니다.

## 주요 스크립트

### `battery_monitor.py`
INA219 전압 센서와 SSD1306 OLED를 사용하여 배터리 상태를 실시간 모니터링합니다.

**기능:**
- 배터리 전압 측정
- 배터리 잔량(SOC) 계산
- 네트워크 상태 표시
- OLED 디스플레이에 정보 출력
- `/dev/shm/jetracer_voltage` 파일에 전압 정보 저장 (다른 스크립트에서 사용)

**사용법:**
```bash
python -m jetracer.diagnostics.battery_monitor
```

**하드웨어 요구사항:**
- INA219 전압 센서 (I2C 주소: 0x42)
- SSD1306 OLED 디스플레이 (I2C 주소: 0x3C, 128x32)

**자동 시작 설정:**
시스템 부팅 시 자동으로 실행되도록 설정하려면 메인 README의 "배터리 모니터 자동 시작 설정" 섹션을 참조하세요.

## low_speed - 저속 보정 도구

저속 영역에서의 스로틀 매핑을 조정하는 실험 도구입니다.

### `interactive.py`
인터랙티브하게 저속 영역의 스로틀 매핑을 조정합니다.

**사용법:**
```bash
python -m jetracer.diagnostics.low_speed.interactive
```

키보드 입력을 통해 실시간으로 저속 영역의 스로틀 응답을 조정할 수 있습니다.

### `pwm_sweep.py`
일정 속도로 PWM 값을 스윕하여 테스트합니다.

**사용법:**
```bash
python -m jetracer.diagnostics.low_speed.pwm_sweep
```

저속 영역에서의 스로틀 응답 특성을 분석할 때 사용합니다.

## 사용 시나리오

1. **배터리 모니터링**: 배터리 상태를 실시간으로 확인하고, 전압 보상 기능이 정상 작동하는지 확인
2. **저속 보정**: 저속 영역에서 차량이 정확하게 움직이도록 스로틀 매핑 조정
3. **성능 최적화**: 다양한 속도에서의 차량 응답 특성 분석

