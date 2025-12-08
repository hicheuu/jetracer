# core - 차량 제어 핵심 클래스

JetRacer의 기본 차량 제어 클래스를 제공합니다.

## 주요 클래스

### `Racecar`
기본 차량 제어 클래스. 스티어링과 스로틀을 제어하는 기본 인터페이스를 제공합니다.

### `NvidiaRacecar`
PCA9685 기반 JetRacer 전용 차량 제어 클래스.

**주요 기능:**
- 스티어링 및 스로틀 제어
- 배터리 전압 보상 (자동 속도 조절)
- ESC 중립점 자동 설정

**사용 예시:**

```python
from jetracer.core import NvidiaRacecar

# 차량 인스턴스 생성
car = NvidiaRacecar()

# 스티어링 제어 (-1.0 ~ 1.0)
car.steering = 0.5  # 오른쪽으로 회전

# 스로틀 제어 (-1.0 ~ 1.0)
car.throttle = 0.3  # 전진 (약 0.6 m/s)

# 정지
car.throttle = 0.0
car.steering = 0.0
```

**설정 방법:**

모든 파라미터는 JSON 설정 파일(`config/nvidia_racecar_config.json`)로 관리됩니다.

```bash
# 설정 파일 생성 (프로젝트 루트의 config 폴더에)
cp config/nvidia_racecar_config.json config/nvidia_racecar_config.json
# 또는 직접 편집
nano config/nvidia_racecar_config.json
```

설정 파일이 없거나 오류가 있으면 기본값이 사용됩니다.

**설정 가능한 속성:**
- `i2c_address`: I2C 주소 (기본값: 64 (0x40))
- `steering.gain`: 스티어링 게인 (기본값: -0.65)
- `steering.offset`: 스티어링 중앙 오프셋 (기본값: 0.22)
- `steering.channel`: 스티어링 채널 (기본값: 0)
- `throttle.gain`: 스로틀 게인 (기본값: 0.8)
- `throttle.offset`: ESC 스로틀 오프셋 (기본값: 0.0)
- `throttle.channel`: 스로틀 채널 (기본값: 1)
- `throttle.neutral`: ESC 중립점 (기본값: 0.12)
- `voltage_compensation.enabled`: 전압 보상 활성화 (기본값: true)
- `voltage_compensation.reference_voltage`: 기준 전압 (기본값: 8.2V)
- `voltage_compensation.ema_alpha`: EMA 필터 alpha (기본값: 0.1)
- `voltage_compensation.gain_min`: 최소 gain (기본값: 1.05)
- `voltage_compensation.gain_max`: 최대 gain (기본값: 1.15)
- `input_to_ms`: 입력값→속도 변환계수 (기본값: 2.0)
- `voltage_compensation`: 배터리 전압 보상 활성화 여부 (기본값: True)
- `reference_voltage`: 보상 기준 전압 (기본값: 8.2V)

## 임포트 방법

```python
# 권장 방법
from jetracer.core import NvidiaRacecar, Racecar

# 레거시 방법 (하위 호환)
from jetracer.nvidia_racecar import NvidiaRacecar
from jetracer.racecar import Racecar
```

