# keyboard drive teleop (throttle + steering)
import sys

from jetracer.nvidia_racecar import NvidiaRacecar

try:
    import msvcrt

    def _getch():
        try:
            return msvcrt.getch().decode("utf-8").lower()
        except UnicodeDecodeError:
            return ""

except ImportError:
    import termios
    import tty

    def _getch():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1).lower()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch


def clamp(value, minimum=-1.0, maximum=1.0):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def main(throttle_step=0.05, steering_step=0.05):
    car = NvidiaRacecar()
    throttle = 0.0
    steering = 0.0
    car.throttle = throttle
    car.steering = steering
    print("Keyboard drive control")
    print("w/s: throttle ±step, a/d: steering ±step, r: reset, q: quit")
    print(f"steps -> throttle:{throttle_step} steering:{steering_step}")
    try:
        while True:
            key = _getch()
            if not key:
                continue
            if key == "w":
                throttle = clamp(throttle + throttle_step, -1.0, 1.0)
            elif key == "s":
                throttle = clamp(throttle - throttle_step, -1.0, 1.0)
            elif key == "a":
                steering = clamp(steering + steering_step)
            elif key == "d":
                steering = clamp(steering - steering_step)
            elif key == "r":
                throttle = 0.0
                steering = 0.0
            elif key == "q":
                break
            else:
                continue

            car.throttle = throttle
            car.steering = steering
            print(f"throttle={throttle:.2f} steering={steering:.2f}")
    except KeyboardInterrupt:
        print("Ctrl+C pressed, stopping")
    finally:
        car.throttle = 0.0
        car.steering = 0.0


if __name__ == "__main__":
    main()


