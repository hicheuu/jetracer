import serial
import serial.tools.list_ports
import json
import numpy as np
import matplotlib.pyplot as plt
import threading
import time

# --------------------------
# Load Calibration JSON
# --------------------------
CALIB_FILE = "mag_calibration.json"

with open(CALIB_FILE, "r") as f:
    calib = json.load(f)

mag_offset_x = calib["mag_offset_x"]
mag_offset_y = calib["mag_offset_y"]
mag_scale_x = calib["mag_scale_x"]
mag_scale_y = calib["mag_scale_y"]

print("=== Loaded Magnetometer Calibration ===")
print(json.dumps(calib, indent=4))


# --------------------------
# Find Serial Port
# --------------------------
def find_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise Exception("No IMU detected")
    print(f"Connecting to {ports[0].device}...")
    return ports[0].device


# --------------------------
# Shared Buffers
# --------------------------
raw_x = []
raw_y = []
cal_x = []
cal_y = []
running = True


# --------------------------
# Serial Thread
# --------------------------
def serial_thread(port):
    global running
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)
    except Exception as e:
        print(f"Serial open error: {e}")
        return

    while running:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            parts = line.split(",")
            if len(parts) < 10:
                continue

            mx = float(parts[7])
            my = float(parts[8])

            # raw 저장
            raw_x.append(mx)
            raw_y.append(my)

            # 보정 적용
            mx_cal = (mx - mag_offset_x) * mag_scale_x
            my_cal = (my - mag_offset_y) * mag_scale_y

            cal_x.append(mx_cal)
            cal_y.append(my_cal)

        except:
            pass

    ser.close()
    print("Serial thread terminated.")


# --------------------------
# Main Visualization
# --------------------------
if __name__ == "__main__":
    port = find_port()
    t = threading.Thread(target=serial_thread, args=(port,))
    t.start()

    plt.ion()
    fig, ax = plt.subplots(1, 2, figsize=(12, 6))

    ax_raw = ax[0]
    ax_cal = ax[1]

    ax_raw.set_title("RAW Magnetometer")
    ax_cal.set_title("CALIBRATED Magnetometer")

    ax_raw.set_xlabel("mx")
    ax_raw.set_ylabel("my")

    ax_cal.set_xlabel("mx_cal")
    ax_cal.set_ylabel("my_cal")

    ax_raw.axis("equal")
    ax_cal.axis("equal")

    print("\nRotate your IMU in all directions...")
    print("This will show RAW vs CALIBRATED magnetic field.\n")

    try:
        while True:
            if len(raw_x) > 5:
                # 최신 1000개만 표시해서 속도 유지
                rx = raw_x[-1000:]
                ry = raw_y[-1000:]
                cx = cal_x[-1000:]
                cy = cal_y[-1000:]

                ax_raw.clear()
                ax_cal.clear()

                ax_raw.scatter(rx, ry, s=7, color="red", alpha=0.6)
                ax_raw.set_title("RAW Magnetometer")
                ax_raw.axis("equal")

                ax_cal.scatter(cx, cy, s=7, color="blue", alpha=0.6)
                ax_cal.set_title("CALIBRATED Magnetometer")
                ax_cal.axis("equal")

                plt.pause(0.05)
            else:
                plt.pause(0.1)

    except KeyboardInterrupt:
        print("\nStopping...")
        running = False
        t.join()
        print("Done.")
