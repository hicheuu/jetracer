from .nvidia_racecar import NvidiaRacecar
print("Calibration: steering offset. Use this to find neutral.")
car = NvidiaRacecar()
try:
    car.steering = 0.0
    car.throttle = 0.0
    print("Set steering_offset in code if wheels veer off-center.")
    input("Press Enter to finish...")
finally:
    car.throttle = 0.0
    car.steering = 0.0
