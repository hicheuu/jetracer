# sensors - 센서 데이터 처리

IMU 등 센서 데이터를 읽고 처리하는 스크립트 모음입니다.

## 주요 스크립트

### `imu_reader.py`
SparkFun 9DoF Razor IMU M0에서 데이터를 읽어 출력합니다.

**사용법:**
```bash
python -m jetracer.sensors.imu_reader
```

시리얼 포트를 통해 IMU 데이터(가속도, 자이로, 자력계)를 실시간으로 읽어 출력합니다.

### `imu_debug.py`
IMU 데이터 디버깅 및 테스트용 스크립트.

### `imu_fusion_heading.py`
IMU 데이터를 융합하여 헤딩(방향) 정보를 계산합니다.

**사용법:**
```bash
python -m jetracer.sensors.imu_fusion_heading
```

### `imu_madgwick.py`
Madgwick 필터를 사용한 IMU 데이터 융합 스크립트.

**사용법:**
```bash
python -m jetracer.sensors.imu_madgwick
```

## 하드웨어 요구사항

- SparkFun 9DoF Razor IMU M0
- USB 시리얼 연결
- `pyserial` 패키지 설치 필요

## 참고

- IMU Firmware는 `9DOF_Razor_IMU/Firmware/` 디렉토리에 있습니다.
- 시리얼 포트는 자동으로 감지되거나 수동으로 선택할 수 있습니다.
- 기본 보레이트: 115200

