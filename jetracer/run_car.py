from .nvidia_racecar import NvidiaRacecar
import time

def main():
    car = NvidiaRacecar()
    print("Starting Jetracer demo. Ctrl+C to stop.")
    try:
        car.steering = 0.0
        car.throttle = 0.0
        time.sleep(0.5)

        print("→ Slight right, forward")
        car.steering = 0.5
        car.throttle = 0.3
        time.sleep(2)

        print("→ Slight left, forward")
        car.steering = -0.5
        car.throttle = 0.3
        time.sleep(2)

        print("→ Stop")
        car.steering = 0.0
        car.throttle = 0.0
        time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        car.throttle = 0.0
        car.steering = 0.0

if __name__ == "__main__":
    main()
