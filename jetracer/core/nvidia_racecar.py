import threading
import traitlets
from adafruit_servokit import ServoKit
import os

from .racecar import Racecar


class NvidiaRacecar(Racecar):
    """JetRacer용 PCA9685 기반 차량 제어 클래스."""

    i2c_address = traitlets.Integer(default_value=0x40)
    steering_gain = traitlets.Float(default_value=-0.65)
    steering_offset = traitlets.Float(default_value=0)
    steering_channel = traitlets.Integer(default_value=0)
    throttle_gain = traitlets.Float(default_value=0.8)
    throttle_channel = traitlets.Integer(default_value=1)
    
    # 전압 보상 관련 설정
    voltage_compensation = traitlets.Bool(default_value=True, help="배터리 전압 보상 기능 활성화 여부")
    reference_voltage = traitlets.Float(default_value=8.4, help="보상의 기준이 되는 완충 전압 (V)")
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.kit = ServoKit(channels=16, address=self.i2c_address)
        self.kit._pca.frequency = 60
        self.steering_motor = self.kit.continuous_servo[self.throttle_channel]
        self.throttle_motor = self.kit.continuous_servo[self.steering_channel]
        self.steering_motor.throttle = 0
        self.throttle_motor.throttle = 0
        
        # 직접 I2C를 초기화하지 않고, `battery_monitor.py`가 /dev/shm에 써놓은
        # 현재 전압 파일을 읽어 전압 보상을 적용합니다 (충돌 방지 및 성능 향상).

        # 전압 상태를 10초마다 출력하는 리포터 스레드 시작
        self._reporter_stop = threading.Event()
        self._reporter_thread = threading.Thread(target=self._voltage_reporter, daemon=True)
        self._reporter_thread.start()

    def _get_voltage_gain(self):
        """파일에서 현재 배터리 전압을 읽어 보정 계수를 계산합니다."""
        if not self.voltage_compensation:
            return 1.0
        # 읽기 로직을 별도 헬퍼로 분리
        current_voltage = self._read_voltage()

        if current_voltage is None:
            return 1.0

        # 비정상적으로 낮은 전압(센서 오류 등)에서는 보정 중단
        if current_voltage < 1.0:
            return 1.0

        # 보정 계수 = 기준전압 / 현재전압
        gain = self.reference_voltage / current_voltage
        return gain

    def _read_voltage(self):
        """/dev/shm에 저장된 전압 값을 읽어 float으로 반환합니다.

        실패 시 `None`을 반환합니다.
        """
        try:
            # /dev/shm은 RAM Disk이므로 매우 빠름
            with open("/dev/shm/jetracer_voltage", "r") as f:
                raw = f.read().strip()
                return float(raw)
        except Exception:
            return None

    def _voltage_reporter(self):
        """10초 간격으로 전압/게인 상태 메시지를 출력하는 백그라운드 루프."""
        while not self._reporter_stop.is_set():
            try:
                current_voltage = self._read_voltage()
                gain = self._get_voltage_gain()

                if current_voltage is None:
                    print("[jetracer][battery] no voltage info in /dev/shm/jetracer_voltage; using gain=%.3f" % (gain,))
                else:
                    print("[jetracer][battery] voltage=%.2fV  gain=%.3f" % (current_voltage, gain))
            except Exception:
                # 리포터에서의 예외는 무시하고 계속 실행
                print("[jetracer][battery] reporter error")

            # 이벤트 대기(중단 가능), 기본 10초
            self._reporter_stop.wait(10)

    @traitlets.observe("steering")
    def _on_steering(self, change):
        self.steering_motor.throttle = change["new"] * self.steering_gain + self.steering_offset

    @traitlets.observe("throttle")
    def _on_throttle(self, change):
        # 1. 전압 보정 계수 계산
        v_gain = self._get_voltage_gain()
        
        # 2. 최종 출력값 계산 (입력값 * 기본게인 * 전압보정)
        final_throttle = change["new"] * self.throttle_gain * v_gain
        
        # 3. 안전 범위 클리핑 (-1.0 ~ 1.0)
        if final_throttle > 1.0:
            final_throttle = 1.0
        elif final_throttle < -1.0:
            final_throttle = -1.0
            
        self.throttle_motor.throttle = final_throttle
