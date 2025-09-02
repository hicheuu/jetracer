import sys, termios, tty, time, os, json
from .nvidia_racecar import NvidiaRacecar

HELP = """
Keyboard Teleop:
  ↑ : throttle +0.1
  ↓ : throttle -0.1
  ← : steering -0.1
  → : steering +0.1
  SPACE : emergency stop (throttle=0)
  c : center steering (0)
  s : SET CENTER (절대값) -> offset = -gain * (현재 steering 입력값), 누적 방지
  w : 현재 offset을 파일에 저장 (~/Jetracer/steering_calib.json)
  r : offset 초기화(0.0)
  p : 현재 상태 출력 (gain/offset, default/loaded/current)
  q : quit
"""

CALIB_PATH = os.path.expanduser("~/Jetracer/steering_calib.json")

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch += sys.stdin.read(2)  # arrow keys
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))

def load_calib():
    if os.path.exists(CALIB_PATH):
        with open(CALIB_PATH, "r") as f:
            d = json.load(f)
        return float(d.get("steering_offset"))
    return None

def save_calib(offset):
    os.makedirs(os.path.dirname(CALIB_PATH), exist_ok=True)
    with open(CALIB_PATH, "w") as f:
        json.dump({"steering_offset": float(offset)}, f, indent=2)
    print(f"[CALIB] saved steering_offset={offset:+.3f} -> {CALIB_PATH}")

def print_status(car, default_offset, loaded_offset):
    print("\n[INFO] --- Steering calibration status ---")
    print(f"  steering_gain    : {car.steering_gain:+.3f}")
    print(f"  steering_offset  : {car.steering_offset:+.3f} (current)")
    print(f"  default_offset   : {default_offset:+.3f}")
    if loaded_offset is None:
        print(f"  loaded_offset    : (none)")
    else:
        print(f"  loaded_offset    : {loaded_offset:+.3f}")
    print("------------------------------------------------\n")

def main():
    car = NvidiaRacecar()

    # 기록용: 실행 시점의 '기본값(코드 내 기본)'
    default_offset = float(getattr(car, "steering_offset", 0.0))

    # 1) 저장 파일이 있으면 '덮어씀' (누적 X)
    loaded_offset = load_calib()
    if loaded_offset is not None:
        prev = car.steering_offset
        car.steering_offset = loaded_offset
        print(f"[CALIB] override: offset default {prev:+.3f} -> loaded {car.steering_offset:+.3f} (source=file)")
    else:
        print(f"[CALIB] no file; using default offset {car.steering_offset:+.3f}")

    steering = 0.0
    throttle = 0.0
    print(HELP)

    try:
        while True:
            ch = getch()
            if ch == 'q':
                break
            elif ch == '\x1b[A':  # up
                throttle = clamp(throttle + 0.1)
            elif ch == '\x1b[B':  # down
                throttle = clamp(throttle - 0.1)
            elif ch == '\x1b[D':  # left
                steering = clamp(steering - 0.1)
            elif ch == '\x1b[C':  # right
                steering = clamp(steering + 0.1)
            elif ch == ' ':
                throttle = 0.0
            elif ch == 'c':
                steering = 0.0
            elif ch == 's':
                # 절대 센터: '누적' 방지 위해 계산 전에 잠시 offset=0 가정하여 입력 해석 명확화
                # (주의: 여기서 사용하는 steering은 '입력값'이라 offset과 무관하지만,
                # 안전하게 이전 오프셋 영향이 없음을 명시적으로 보장)
                prev_offset = car.steering_offset
                # 입력 steering(표시값)을 센터로 간주 → offset = -gain * steering
                new_offset = - car.steering_gain * steering
                car.steering_offset = new_offset
                print(f"\n[CALIB] set-center: steering_in={steering:+.2f} "
                      f"=> offset {prev_offset:+.3f} -> {new_offset:+.3f} (gain={car.steering_gain:+.3f})")
            elif ch == 'w':
                save_calib(car.steering_offset)
            elif ch == 'r':
                prev = car.steering_offset
                car.steering_offset = 0.0
                print(f"\n[CALIB] offset reset: {prev:+.3f} -> 0.000")
            elif ch == 'p':
                print_status(car, default_offset, loaded_offset)

            # 적용
            car.steering = steering
            car.throttle = throttle
            sys.stdout.write(f"\rsteer={steering:+.2f}  throttle={throttle:+.2f}   ")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        car.throttle = 0.0
        car.steering = 0.0
        print("\nStopped.")

if __name__ == "__main__":
    main()

