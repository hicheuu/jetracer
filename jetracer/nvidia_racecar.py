from .racecar import Racecar
import traitlets
from adafruit_servokit import ServoKit
import threading

class NvidiaRacecar(Racecar):
    # 하드웨어/튜닝 파라미터
    i2c_address      = traitlets.Integer(default_value=0x40)  # PCA9685 기본 주소
    steering_gain    = traitlets.Float(default_value=-0.65)    # 방향 반대면 부호 바꾸기
    steering_offset  = traitlets.Float(default_value=0.0)      # 직진 보정
    steering_channel = traitlets.Integer(default_value=0)      # 서보 채널
    throttle_gain    = traitlets.Float(default_value=0.8)      # 속도 감도
    throttle_channel = traitlets.Integer(default_value=1)      # ESC/연속서보 채널

    # 내부 상태(소프트 스타트용)
    status = 0.0
    target = 0.0
    timer  = None

    def __init__(self, *args, **kwargs):
        super(NvidiaRacecar, self).__init__(*args, **kwargs)
        self.kit = ServoKit(channels=16, address=self.i2c_address)
        self.kit._pca.frequency = 60  # 서보 50~60Hz
        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.throttle_channel]
        self.steering_motor.throttle = 0.0
        self.throttle_motor.throttle = 0.0

    @traitlets.observe('steering')
    def _on_steering(self, change):
        val = change['new'] * self.steering_gain + self.steering_offset
        # continuous_servo.throttle: -1.0 ~ 1.0
        self.steering_motor.throttle = max(-1.0, min(1.0, val))

    @traitlets.observe('throttle')
    def _on_throttle(self, change):
        self.target = max(-1.0, min(1.0, change['new'] * self.throttle_gain))
        # 데드존
        if -0.1 < self.target < 0.1:
            self.status = 0.0
            self.throttle_motor.throttle = 0.0
            # 진행 중 타이머 있으면 정리
            if self.timer:
                self.timer.cancel()
                self.timer = None
        elif (self.target < self.status and self.status > 0.0) or (self.target > self.status and self.status < 0.0):
            # 동일 방향 감속은 즉시 반영
            self.status = self.target
            self.throttle_motor.throttle = self.status
            if self.timer:
                self.timer.cancel()
                self.timer = None
        else:
            # 가속/역방향 진입은 부드럽게
            self._soft_start()

    def _soft_start(self):
        step = 0.18
        period = 1.2  # sec
        if self.status < self.target:
            self.status = min(self.status + step, self.target)
        elif self.status > self.target:
            self.status = max(self.status - step, self.target)

        self.throttle_motor.throttle = self.status

        if abs(self.status - self.target) > 1e-6:
            if self.timer:
                self.timer.cancel()
            self.timer = threading.Timer(period, self._soft_start)
            self.timer.daemon = True
            self.timer.start()
