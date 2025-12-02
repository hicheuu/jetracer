# keyboard throttle teleop
import sys

from jetracer.core import NvidiaRacecar

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


def main(step=0.001):
    car = NvidiaRacecar()
    throttle = 0.0
    car.throttle = throttle
    print("Keyboard throttle control")
    print("w: +step, s: -step, r: reset, q: quit, step =", step)
    try:
        while True:
            key = _getch()
            if not key:
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
            car.throttle = throttle
            print(f"throttle={throttle:.3f}")
    except KeyboardInterrupt:
        print("Ctrl+C pressed, stopping")
    finally:
        car.throttle = 0.0
        car.steering = 0.0


if __name__ == "__main__":
    main()


