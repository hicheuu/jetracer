from jetracer.core import NvidiaRacecar


def main():
    car = NvidiaRacecar()
    print("수동 스로틀 입력 (q로 종료). 범위 -1.0 ~ 1.0")
    try:
        while True:
            raw = input("throttle> ").strip()
            if raw.lower() in ("q", "quit", "exit"):
                break
            try:
                value = float(raw)
            except ValueError:
                print("숫자를 입력하세요.")
                continue

            value = max(-1.0, min(1.0, value))
            car.throttle = value
            print(f"적용: {value:.2f}")
    except KeyboardInterrupt:
        print("\nCtrl+C 입력 – 종료")
    finally:
        car.throttle = 0.0
        car.steering = 0.0


if __name__ == "__main__":
    main()

