# JetRacer Unified Controller

NVIDIA JetRacer AI Kit를 위한 통합 제어 시스템입니다. 이 프로젝트는 **Multiplexer(Mux)** 구조를 통해 조이스틱과 UDP 제어를 유연하게 관리하며, 실시간 속도 보정 기능을 제공합니다.

> [!TIP]
> 처음 시작하시나요? [설치 가이드(Setup Guide)](./docs/setup.md)를 먼저 확인하여 환경을 구축하세요.

## 🚀 실행 방법

모든 제어 시스템(`mux`, `joystick`, `udp_recv`)을 통합 실행합니다.

```bash
# 기본 실행 (최고속도 물리 스로틀 0.20 설정)
python3 -m jetracer.runner --speed5-throttle 0.20
```

### 주요 실행 인자 (Arguments)
- `--speed5-throttle`: 최고 목표 물리 출력값 (0.0~1.0)
- `--log-calibration`: 캘리브레이션용 데이터 로깅 활성화
- `--auto-calibrate`: 실시간 실제 속도 기반 스로틀 자동 보정 활성화
- `--target-velocity`: 자동 보정 시 목표 속도 (m/s)

---

## 🎮 컨트롤러 조작법

`runner.py` 실행 중 조이스틱 버튼을 통해 제어할 수 있습니다.

### 모드 전환
- **Y 버튼**: **Joystick Mode** ↔ **UDP Mode** 전환

### 실시간 물리 스로틀 튜닝 (Manual Calibration)
주행 중 `SPEED_5_PHYS` 값을 미세 조정하여 실제 차량 속도를 제어합니다.
- **RB 버튼**: 스로틀 타겟 **증가** (+0.01 / +0.001)
- **LB 버튼**: 스로틀 타겟 **감소** (-0.01 / -0.001)
- *현재 모드(Joystick/UDP)에 따라 보정 단위가 자동으로 변경됩니다.*

### 기타 제어
- **A 버튼**: 전진/후진 모드 토글 (필요 시)
- **B 버튼**: 긴급 정지 (E-Stop) 토글

---

## 📈 속도 캘리브레이션 및 분석

배터리 전압 저하나 모터 발열로 인해 발생하는 속도 드리프트를 해결하는 도구들입니다.

### 1단계: 기록 및 시각화
주행 데이터를 CSV로 기록하고 그래프로 시각화하여 "어느 시점에 보정이 필요한지" 분석할 수 있습니다.

```bash
# 로깅 모드로 주행
python3 -m jetracer.runner --log-calibration

# 그래프 그리기
python3 jetracer/tools/plot_calibration.py
```
- **그래프 해석**: 주황색(설정값)과 초록색(실제 속도) 선을 비교하며 튜닝 타이밍을 확인하세요. 생성된 `calibration_result.png`를 참고합니다.

### 2단계: 자동 보정 (Auto-Correction)
분석된 데이터를 바탕으로 알고리즘이 스스로 스로틀을 조절하게 합니다.

```bash
# 5.0m/s 속도를 유지하도록 자동 보정
python3 -m jetracer.runner --auto-calibrate --target-velocity 5.0
```
- 차량 명령 속도가 4.5 이상일 때만 작동하며, 오차가 발생하면 스스로 `speed5_up/down`을 수행합니다.

---

## 📁 프로젝트 구조
- `jetracer/core/`: 하드웨어 추상화 및 모터 제어
- `jetracer/mux/`: 입력 소스(UDP/Joystick) 통합 및 중재
- `jetracer/tools/`: 캘리브레이션 데이터 시각화 도구
- `config/`: udev 규칙 및 차량 설정
