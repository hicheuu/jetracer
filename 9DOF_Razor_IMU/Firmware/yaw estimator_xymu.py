#!/usr/bin/env python3
"""
Quaternion Yaw Estimator (DMP-based, 10-field format)

- SparkFun 9DoF Razor IMU M0
- Uses DMP quaternion only (no mag, no gyro integration)
- Applies fixed quaternion offset (installation calibration)
"""

import serial
import serial.tools.list_ports
import time
import math
import numpy as np

BAUD_RATE = 115200

# ==========================================
# Quaternion Utils
# ==========================================
def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=float)

def quat_to_yaw(q):
    w, x, y, z = q
    siny = 2*(w*z + x*y)
    cosy = 1 - 2*(y*y + z*z)
    yaw = math.degrees(math.atan2(siny, cosy))
    if yaw < 0:
        yaw += 360
    return yaw

# ==========================================
# Installation Quaternion Offset
# ==========================================
# 네가 실험으로 얻은 값 유지
q_offset = np.array([0.0180163, 0.9926568, -0.11775514, 0.02101488])
q_offset = q_offset / np.linalg.norm(q_offset)

# ==========================================
# Serial Utils
# ==========================================
def find_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No IMU detected")
    for p in ports:
        if "ACM" in p.device or "USB" in p.device:
            return p.device
    return ports[0].device

def parse_imu_data_10(line):
    """
    Expected 10-field format:
    [0] ax
    [1] ay
    [2] az
    [3] gx
    [4] gy
    [5] gz
    [6] qw
    [7] qx
    [8] qy
    [9] qz
    """
    try:
        s = line.strip()
        if s.startswith("#"):
            s = s.strip("#")
        if "XYMU=" in s:
            s = s.split("XYMU=", 1)[1]

        v = [float(x) for x in s.split(",")]
        if len(v) != 10:
            return None

        return {
            "qw": v[6],
            "qx": v[7],
            "qy": v[8],
            "qz": v[9],
        }
    except:
        return None

# ==========================================
# Main
# ==========================================
def main():
    print("=" * 60)
    print("DMP Quaternion Yaw (10-field, patched)")
    print("=" * 60)

    port = find_port()
    print(f"[IMU] port = {port}")

    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    time.sleep(1.0)

    print("\n📡 IMU streaming (Ctrl+C to stop)\n")

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            d = parse_imu_data_10(line)
            if not d:
                continue

            q_raw = np.array([d["qw"], d["qx"], d["qy"], d["qz"]], dtype=float)
            q_raw = q_raw / np.linalg.norm(q_raw)

            q_corr = quat_mul(q_offset, q_raw)
            q_corr = q_corr / np.linalg.norm(q_corr)

            yaw = quat_to_yaw(q_corr)

            print(f"\rYaw: {yaw:6.2f}°", end="", flush=True)

    except KeyboardInterrupt:
        print("\n\n👋 종료")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
