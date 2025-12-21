# keyboard drive teleop (throttle + steering)
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


def clamp(value, minimum=-1.0, maximum=1.0):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def _compute_throttle_cmd(throttle_input, throttle_scale=0.125):
    """
    Convert logical throttle (-1..1) into ESC command with neutral offset
    matching joystick.py behavior.
    """
    ESC_NEUTRAL = 0.12
    REVERSE_START = -0.1

    if throttle_input > 0:
        cmd = ESC_NEUTRAL + throttle_input * (1.0 - ESC_NEUTRAL) * throttle_scale
    elif throttle_input < 0:
        cmd = REVERSE_START + throttle_input * (1.0 - abs(REVERSE_START)) * throttle_scale
    else:
        cmd = ESC_NEUTRAL

    return clamp(cmd, -1.0, 1.0)


def main(throttle_step=0.001, steering_step=0.05, throttle_scale=0.165):
    car = NvidiaRacecar()
    throttle_input = 0.2  # logical -1..1
    steering = 0.0
    car.throttle = _compute_throttle_cmd(throttle_input, throttle_scale)
    car.steering = steering
    print("Keyboard drive control")
    print("w/s: throttle ±step, a/d: steering ±step, r: reset, q: quit")
    print(f"steps -> throttle:{throttle_step} steering:{steering_step}")
    print(f"ESC neutral mapping enabled (scale={throttle_scale})")
    try:
        while True:
            key = _getch()
            if not key:
                continue
            if key == "w":
                throttle_input = clamp(throttle_input + throttle_step, -1.0, 1.0)
            elif key == "s":
                throttle_input = clamp(throttle_input - throttle_step, -1.0, 1.0)
            elif key == "a":
                steering = clamp(steering + steering_step)
            elif key == "d":
                steering = clamp(steering - steering_step)
            elif key == "r":
                throttle_input = 0.0
                steering = 0.0
            elif key == "q":
                break
            else:
                continue

            car.throttle = _compute_throttle_cmd(throttle_input, throttle_scale)
            car.steering = steering
            print(f"throttle_in={throttle_input:.4f} throttle_cmd={car.throttle:.4f} steering={steering:.2f}")
    except KeyboardInterrupt:
        print("Ctrl+C pressed, stopping")
    finally:
        car.throttle = _compute_throttle_cmd(0.0, throttle_scale)
        car.steering = 0.0


if __name__ == "__main__":
    main()


