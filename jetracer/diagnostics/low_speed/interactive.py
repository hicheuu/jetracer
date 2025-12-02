# scripts/low_speed_check.py
try:
    import msvcrt

    getch = msvcrt.getch
except ImportError:
    from getch import getch  # Linux

from jetracer.core import NvidiaRacecar


class LowSpeedMapper:
    def __init__(self, min_output=0.04, low_zone=0.3):
        self.min_output = min_output
        self.low_zone = low_zone

    def scaled_throttle(self, raw):
        raw = max(-1.0, min(1.0, raw))
        if raw == 0.0:
            return 0.0

        sign = 1 if raw > 0 else -1
        mag = abs(raw)

        if mag < self.low_zone:
            mag = self.min_output + (mag / self.low_zone) ** 2.5 * (1 - self.min_output)
        else:
            mag = self.min_output + (mag - self.low_zone) * (1 - self.min_output) / (1 - self.low_zone)

        return sign * min(1.0, mag)


def interactive_low_speed(car, mapper, start=0.0, step=0.01):
    throttle = start
    try:
        scaled = mapper.scaled_throttle(throttle)
        car.throttle = scaled
        print(f"시작 스로틀: 입력 {throttle:.2f} -> 적용 {scaled:.2f}")

        while True:
            try:
                key = getch()
            except UnicodeDecodeError:
                continue

            if key == "w":
                throttle = min(throttle + step, 1.0)
            elif key == "s":
                throttle = max(throttle - step, -1.0)
            elif key == "r":
                throttle = 0.0
            elif key == "q":
                break
            else:
                continue

            scaled = mapper.scaled_throttle(throttle)
            car.throttle = scaled
            print(f"입력 {throttle:.2f} -> 적용 {scaled:.2f}")
    except KeyboardInterrupt:
        print("Ctrl+C 입력 – 모터 정지")
    finally:
        car.throttle = 0.0


if __name__ == "__main__":
    car = NvidiaRacecar()
    mapper = LowSpeedMapper(min_output=0.08, low_zone=0.3)
    interactive_low_speed(car, mapper)


