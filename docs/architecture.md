# JetRacer 아키텍처 개요

이 문서는 JetRacer 프로젝트의 전체 구조를 설명합니다. AI가 프롬프트를 입력했을 때 전체 구조를 바로 이해하고 시작할 수 있도록 작성되었습니다.

## 프로젝트 구조

```
jetracer/
├── runner.py              # 메인 진입점: 모든 제어 프로세스를 통합 실행
├── core/                  # 차량 제어 핵심 클래스
│   ├── racecar.py        # 기본 Racecar 추상 클래스
│   └── nvidia_racecar.py # PCA9685 기반 JetRacer 구현체
├── mux/                   # 멀티플렉서 시스템 (제어 입력 소스 선택)
│   ├── mux.py            # 제어 중심: 입력 선택 및 차량 제어
│   ├── joystick.py       # 조이스틱 입력 처리
│   └── udp_recv.py       # UDP 네트워크 입력 처리
└── config/                # 설정 파일 (템플릿)
    └── nvidia_racecar_config.json
```

## 핵심 실행 흐름

### 1. 프로세스 통합 실행 (`runner.py`)

**`runner.py`는 모듈로 실행되어 `mux/` 디렉토리의 모든 코드를 동시에 실행합니다.**

- **MUX 프로세스** (`mux/mux.py`): 제어의 중심 루프
  - UDS 소켓(`/tmp/jetracer_ctrl.sock`)을 통해 조이스틱과 UDP 입력을 수신
  - 현재 모드(joystick/udp)에 따라 입력 소스를 선택
  - `NvidiaRacecar` 인스턴스를 통해 실제 하드웨어 제어

- **Joystick 프로세스** (`mux/joystick.py`): 게임패드 입력 처리
  - evdev를 통해 조이스틱 입력 읽기
  - 정규화된 제어 값(-1.0 ~ 1.0)을 UDS로 전송

- **UDP 프로세스** (`mux/udp_recv.py`): 네트워크 입력 처리
  - UDP 포트 5555에서 외부 제어 명령 수신
  - 속도 명령(0.0 ~ 5.0)과 조향 명령을 UDS로 전송
  - 실시간 속도 기반 자동 보정 기능 포함

**실행 방법:**
```bash
python -m jetracer.runner [옵션]
```

## 설정 파일 관리 (`config/`)

### 중요 원칙

1. **경로 고정**: `core/nvidia_racecar.py`에서 설정 파일 경로를 수정하지 않도록 함
   - 현재 경로: `{project_root}/config/nvidia_racecar_config.json`
   - 이 레포지토리는 **다수의 RC 차량에 코드를 배포**하기 위한 것이므로, 각 차량별로 설정 파일만 복사하여 사용

2. **템플릿 역할**: `jetracer/config/nvidia_racecar_config.json`은 **포맷이 바뀌었을 때 쉽게 복사하기 위한 템플릿**입니다
   - 실제 사용되는 설정 파일은 프로젝트 루트의 `config/` 디렉토리에 위치
   - 새로운 파라미터가 추가될 때는 **반드시 이 템플릿 파일에도 추가**해야 함

3. **설정 파일 구조**:
```json
{
  "i2c_address": 64,
  "steering": {
    "gain": -0.65,
    "offset": 0.0,
    "channel": 1,
    "throttle_gain_left": 0.0,
    "throttle_gain_right": 0.0
  },
  "throttle": {
    "gain": 1.0,
    "channel": 0,
    "neutral": 0.21,
    "speed5_throttle": 0.35,
    "reverse_start": -0.1,
    "reverse_gain": 0.7
  },
  "input_to_ms": 2.0
}
```

### 설정 파일 패치 가이드

새로운 파라미터를 추가할 때:

1. `core/nvidia_racecar.py`의 `load_config()` 함수에서 새 파라미터 로드 로직 추가
2. **반드시** `jetracer/config/nvidia_racecar_config.json` 템플릿에도 해당 필드 추가
3. 프로젝트 루트의 `config/nvidia_racecar_config.json`에도 추가 (실제 사용 파일)

## 코드 수정 원칙

### ⚠️ 중요: 파라미터 값 수정 금지

**로직을 수정할 때는 수정을 요청한 내용에 대해서만 수정하고, 다른 코드의 파라미터값을 수정하지 않도록 합니다.**

- 기본값, 상수, 임계값 등은 변경하지 않음
- 요청된 기능만 수정하고 기존 동작은 유지
- 파라미터 조정이 필요한 경우 명시적으로 요청받았을 때만 수행

## 통신 프로토콜

### UDS (Unix Domain Socket)

모든 프로세스 간 통신은 `/tmp/jetracer_ctrl.sock`을 통해 이루어집니다.

**메시지 포맷:**
```json
{
  "src": "joystick" | "udp" | "auto",
  "steer": -1.0 ~ 1.0,
  "throttle": -1.0 ~ 1.0,  // joystick 모드
  "speed": 0.0 ~ 5.0,      // UDP 모드
  "obs_speed": float,      // UDP 모드: 실제 관측 속도
  "event": "toggle" | "estop" | "speed5_up" | "speed5_down" | ...
}
```

## 제어 모드

- **Joystick 모드**: 조이스틱 입력을 직접 차량에 전달
- **UDP 모드**: 외부 네트워크에서 속도 명령(0.0~5.0)을 받아 차량 제어
- **모드 전환**: 조이스틱 Y 버튼으로 전환

## 주요 클래스

### `NvidiaRacecar` (`core/nvidia_racecar.py`)

PCA9685 기반 하드웨어 제어 클래스:
- 정규화된 입력(-1.0 ~ 1.0)을 물리적 PWM 신호로 변환
- 설정 파일에서 파라미터 로드
- 후진 시작점 점프(discontinuous reverse) 로직 포함

**중요 속성:**
- `steering`: 조향 입력 (-1.0 ~ 1.0)
- `throttle`: 스로틀 입력 (-1.0 ~ 1.0)
- `physical_throttle`: 실제 ESC에 전달되는 물리적 값 (읽기 전용)

## 배포 시나리오

1. 코드는 모든 차량에 동일하게 배포
2. 각 차량별로 `config/nvidia_racecar_config.json`만 차량 특성에 맞게 수정
3. `core/nvidia_racecar.py`의 경로 로직은 변경하지 않음 (다중 배포 호환성 유지)

