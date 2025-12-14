#!/usr/bin/env python3
import serial
import math
import time
import struct
import os

PORT = "/dev/ttyIMU"
BAUD = 115200

TARGET_HZ = 30.0
WINDOW_DT = 1.0 / TARGET_HZ

SHM_PATH = "/dev/shm/jetracer_heading_delta"
FMT = "ffI"  # heading_diff(rad), heading_dt(sec), seq
SIZE = struct.calcsize(FMT)


def quat_to_yaw(qw, qx, qy, qz):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def wrap(a):
    if a > math.pi:
        a -= 2 * math.pi
    elif a < -math.pi:
        a += 2 * math.pi
    return a


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(1)

    fd = os.open(SHM_PATH, os.O_CREAT | os.O_RDWR)
    os.ftruncate(fd, SIZE)

    prev_yaw = None
    prev_t = None

    acc_dyaw = 0.0
    acc_dt = 0.0
    seq = 0

    last_emit = time.monotonic()

    print("[IMU] yaw delta writer (30Hz accumulated) started")

    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line.startswith("#XYMU="):
            continue

        d = line.split("=")[1].split(",")
        if len(d) < 7:
            continue

        qw, qx, qy, qz = map(float, d[3:7])
        yaw = quat_to_yaw(qw, qx, qy, qz)
        now = time.monotonic()

        if prev_yaw is None:
            prev_yaw = yaw
            prev_t = now
            continue

        dt = now - prev_t
        if dt <= 0:
            continue

        dyaw = wrap(yaw - prev_yaw)

        prev_yaw = yaw
        prev_t = now

        # 🔵 누적
        acc_dyaw += dyaw
        acc_dt += dt

        # 🔴 30Hz 윈도우 도달 시에만 write
        if acc_dt >= WINDOW_DT:
            seq += 1
            os.lseek(fd, 0, os.SEEK_SET)
            os.write(fd, struct.pack(FMT, acc_dyaw, acc_dt, seq))

            # 누적 리셋
            acc_dyaw = 0.0
            acc_dt = 0.0
            last_emit = now


if __name__ == "__main__":
    main()
