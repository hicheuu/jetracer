#!/usr/bin/env python3
"""
Mag Heading + 1D Kalman + Quaternion Offset Correction
- 자력계 기반 절대방향
- 쿼터니언 설치각 보정(q_offset)
- 자이로 사용 안함
"""

import serial
import serial.tools.list_ports
import time
import math
import json
import os
import numpy as np

BAUD_RATE = 115200
CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "mag_calibration.json")

# ==========================================
# 1) Quaternion Utils
# ==========================================
def quat_conj(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z], dtype=float)

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=float)

def quat_to_euler_zyx(q):
    w, x, y, z = q

    # roll
    sinr = 2*(w*x + y*z)
    cosr = 1 - 2*(x*x + y*y)
    roll = math.degrees(math.atan2(sinr, cosr))

    # pitch
    sinp = 2*(w*y - z*x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    # yaw
    siny = 2*(w*z + x*y)
    cosy = 1 - 2*(y*y + z*z)
    yaw = math.degrees(math.atan2(siny, cosy))

    if yaw < 0:
        yaw += 360
    return roll, pitch, yaw

# ==========================================
# 2) q_offset 설정
# ==========================================
# 네가 실제로 얻은 보정값 직접 입력:
# q_calib = [0.01999816, 0.52183147, 0.85172901, 0.04300796]
# q_offset = conjugate(q_calib)
q_offset = np.array([ 0.0180163, 0.9926568, -0.11775514, 0.02101488])
# q_offset = np.array([-0.00454261, 0.78627062, 0.61766535, 0.01572884])

# ==========================================
# 3) Kalman Filter
# ==========================================
class SimpleKalman1D:
    def __init__(self, R=0.1, Q=0.001):
        self.R = R
        self.Q = Q
        self.P = 1.0
        self.x = 0.0
        self.first_run = True

    def update(self, measurement):
        if self.first_run:
            self.x = measurement
            self.first_run = False
            return self.x

        self.P = self.P + self.Q

        delta = measurement - self.x
        if delta > 180: delta -= 360
        elif delta < -180: delta += 360

        K = self.P / (self.P + self.R)
        self.x += K * delta
        self.P = (1 - K) * self.P

        if self.x < 0: self.x += 360
        elif self.x >= 360: self.x -= 360

        return self.x

# ==========================================
# 4) Utility
# ==========================================
def load_calibration():
    if os.path.exists(CALIBRATION_FILE):
        try:
            with open(CALIBRATION_FILE, 'r') as f:
                return json.load(f)
        except:
            pass
    return {
        "mag_offset_x": 0.0,
        "mag_offset_y": 0.0,
        "mag_scale_x": 1.0,
        "mag_scale_y": 1.0
    }

def find_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise Exception("No IMU detected")
    return ports[0].device

def parse_imu_data(line):
    parts = line.split(',')
    if len(parts) >= 15:
        return {
            "mx": float(parts[7]),
            "my": float(parts[8]),
            "qw": float(parts[10]),
            "qx": float(parts[11]),
            "qy": float(parts[12]),
            "qz": float(parts[13]),
            "hdg_fw": float(parts[14])
        }
    return None

def simple_heading(mx, my, cal):
    mx = (mx - cal["mag_offset_x"]) * cal["mag_scale_x"]
    my = (my - cal["mag_offset_y"]) * cal["mag_scale_y"]

    heading = math.degrees(math.atan2(my, mx))
    if heading < 0:
        heading += 360
    return heading

# ==========================================
# 5) Main
# ==========================================
def main():
    print("=" * 60)
    print("Quaternion Offset + Mag Heading + 1D Kalman")
    print("=" * 60)

    cal = load_calibration()
    port = find_port()
    ser = serial.Serial(port, BAUD_RATE, timeout=1)

    kf = SimpleKalman1D(R=0.2, Q=0.005)

    print("\n📡 IMU streaming...\n")

    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        d = parse_imu_data(line)
        if not d:
            continue

        # -----------------------------
        # ① Quaternion correction
        # -----------------------------
        q_raw = np.array([d["qw"], d["qx"], d["qy"], d["qz"]], dtype=float)
        q_raw = q_raw / np.linalg.norm(q_raw)

        q_corr = quat_mul(q_offset, q_raw)
        q_corr = q_corr / np.linalg.norm(q_corr)

        roll, pitch, yaw_q = quat_to_euler_zyx(q_corr)

        # -----------------------------
        # ② Magnetometer heading
        # -----------------------------
        h_mag = simple_heading(d["mx"], d["my"], cal)

        # -----------------------------
        # ③ Kalman Filter
        # -----------------------------
        h_filtered = kf.update(h_mag)

        print(
            f"\rQ-Yaw:{yaw_q:6.1f}° | "
            f"Mag:{h_mag:6.1f}° | "
            f"Filt:{h_filtered:6.1f}°",
            end=""
        )


if __name__ == "__main__":
    main()
