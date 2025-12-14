#!/usr/bin/env python3
import serial
import math
import time
import struct
import os

PORT = "/dev/ttyIMU"
BAUD = 115200

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
    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=0,              # 🔴 논블로킹
        inter_byte_timeout=0
    )
    time.sleep(1)

    fd = os.open(SHM_PATH, os.O_CREAT | os.O_RDWR)
    os.ftruncate(fd, SIZE)

    prev_yaw = None
    prev_t = None
    seq = 0

    print("[IMU] yaw delta writer started")

    while True:
        if not ser.in_waiting:
            time.sleep(0.001)
            continue

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

        dt_real = now - prev_t
        if dt_real <= 0:
            continue

        dyaw = wrap(yaw - prev_yaw)

        prev_yaw = yaw
        prev_t = now
        seq += 1

        os.lseek(fd, 0, os.SEEK_SET)
        os.write(fd, struct.pack(FMT, dyaw, dt_real, seq))

        # 🔍 디버그 (느리면 여기서 바로 보임)
        # print(f"dyaw={dyaw:+.4f} rad dt={dt_real*1000:.1f} ms")

if __name__ == "__main__":
    main()
