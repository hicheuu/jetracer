#!/usr/bin/env python3
import serial
import math
import time
import struct
import os

PORT = "/dev/ttyACM0"
BAUD = 115200

TARGET_HZ = 30.0
WINDOW_DT = 1.0 / TARGET_HZ

# ===== 방어 파라미터 =====
MAX_YAW_RATE = 6.0     # rad/s
MAX_DT = 0.10          # sec

SHM_PATH = "/dev/shm/jetracer_heading_delta"
FMT = "ffI"            # heading_diff(rad), heading_dt(sec), seq
SIZE = struct.calcsize(FMT)

# =========================
# Quaternion utils
# =========================
def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )

def quat_delta_yaw(q_prev, q_now):
    # q_delta = q_prev^-1 ⊗ q_now
    qd = quat_mul(quat_conj(q_prev), q_now)
    qw, qx, qy, qz = qd

    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny, cosy)

# =========================
# Main
# =========================
def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(1)

    fd = os.open(SHM_PATH, os.O_CREAT | os.O_RDWR)
    os.ftruncate(fd, SIZE)

    prev_q = None
    prev_t = None

    acc_dyaw = 0.0
    acc_dt = 0.0
    seq = 0

    print("[IMU] yaw delta writer (30Hz, quaternion-delta) started")

    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not (line.startswith("#XYMU=") and line.endswith("#")):
            continue

        d = line[6:-1].split(",")
        if len(d) < 7:
            continue

        try:
            qw, qx, qy, qz = map(float, d[3:7])
        except ValueError:
            continue

        q_now = (qw, qx, qy, qz)
        now = time.monotonic()

        # ===== 초기화 =====
        if prev_q is None:
            prev_q = q_now
            prev_t = now
            continue

        dt = now - prev_t

        # ===== dt 방어 + 재동기화 =====
        if dt <= 0.0 or dt > MAX_DT:
            acc_dyaw = 0.0
            acc_dt = 0.0
            prev_q = q_now
            prev_t = now
            continue

        # ⭐ 핵심: quaternion delta 기반 yaw
        dyaw = quat_delta_yaw(prev_q, q_now)

        # ===== 물리 한계 방어 =====
        if abs(dyaw) > MAX_YAW_RATE * dt:
            acc_dyaw = 0.0
            acc_dt = 0.0
            prev_q = q_now
            prev_t = now
            continue

        # ===== 정상 프레임 =====
        prev_q = q_now
        prev_t = now

        acc_dyaw += dyaw
        acc_dt += dt

        # ===== 30Hz emit =====
        if acc_dt >= WINDOW_DT:
            seq += 1
            os.lseek(fd, 0, os.SEEK_SET)
            os.write(fd, struct.pack(FMT, acc_dyaw, acc_dt, seq))
            acc_dyaw = 0.0
            acc_dt = 0.0

if __name__ == "__main__":
    main()
