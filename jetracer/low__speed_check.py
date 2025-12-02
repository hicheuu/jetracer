# scripts/low_speed_check.py
import time
from jetracer.nvidia_racecar import NvidiaRacecar

class LowSpeedMapper:
    def __init__(self, min_duty=0.08, low_zone=0.3, throttle_gain=0.5):
        self.min_duty = min_duty
        self.low_zone = low_zone
        self.throttle_gain = throttle_gain

    def pwm_from_throttle(self, throttle):
        value = throttle * self.throttle_gain
        if value == 0:
            return 0, 0
        sign = 1 if value > 0 else -1
        mag = min(abs(value), 1.0)

        if mag < self.low_zone:
            mag = self.min_duty + (mag / self.low_zone) ** 2 * (1 - self.min_duty)
        else:
            mag = self.min_duty + (mag - self.low_zone) * (1 - self.min_duty) / (1 - self.low_zone)

        pwm = int(0xFFFF * mag)
        return pwm, sign

def apply_pwm(car, pwm, direction):
    channels = car.motor._pca.channels
    if pwm == 0:
        for idx in range(8):
            channels[idx].duty_cycle = 0
        return

    if direction > 0:
        channels[0].duty_cycle = pwm
        channels[1].duty_cycle = 0xFFFF
        channels[2].duty_cycle = 0
        channels[3].duty_cycle = 0
        channels[4].duty_cycle = pwm
        channels[7].duty_cycle = pwm
        channels[6].duty_cycle = 0xFFFF
        channels[5].duty_cycle = 0
    else:
        channels[0].duty_cycle = pwm
        channels[1].duty_cycle = 0
        channels[2].duty_cycle = 0xFFFF
        channels[3].duty_cycle = pwm
        channels[4].duty_cycle = 0
        channels[7].duty_cycle = pwm
        channels[6].duty_cycle = 0
        channels[5].duty_cycle = 0xFFFF

def sweep_low_speed(car, mapper, start=0.05, stop=0.30, step=0.02, dwell=1.5):
    throttle = start
    while throttle <= stop:
        pwm, direction = mapper.pwm_from_throttle(throttle)
        apply_pwm(car, pwm, direction)
        print(f"throttle={throttle:.2f} pwm={pwm} dir={direction}")
        time.sleep(dwell)
        throttle += step
    apply_pwm(car, 0, 0)

if __name__ == "__main__":
    car = NvidiaRacecar()
    mapper = LowSpeedMapper(min_duty=0.08, low_zone=0.3, throttle_gain=0.5)
    sweep_low_speed(car, mapper)
