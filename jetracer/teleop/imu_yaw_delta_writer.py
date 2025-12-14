#!/usr/bin/env python3
import serial
import math
import time
import struct
import os

PORT = "/dev/ttyIMU"
BAUD = 115200
HZ = 30.0
DT = 1.0 / HZ

SHM_PATH = "/dev/shm/jetracer_heading_delta"
FMT = "fffI"  # heading_diff, heading_dt, dummy_voltage, seq (padding 포함)
SIZE = struct.calcsize(FMT)


def quat_to_yaw(qw, qx, qy, qz):
    siny = 2.0 * (qw*qz + qx*qy)
    cosy = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny, cosy)


def wrap(a):
    if a > math.pi:
        a -= 2*math.pi
    elif a < -math.pi:
        a += 2*math.pi
    return a


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)

    fd = os.open(SHM_PATH, os.O_CREAT | os.O_RDWR)
    os.ftruncate(fd, SIZE)

    prev_yaw = None
    seq = 0

    next_t = time.monotonic()

    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line.startswith("#XYMU="):
            continue

        d = line.split("=")[1].split(",")
        if len(d) < 7:
            continue

        qw, qx, qy, qz = map(float, d[3:7])
        yaw = quat_to_yaw(qw, qx, qy, qz)

        if prev_yaw is None:
            prev_yaw = yaw
            continue

        dyaw = wrap(yaw - prev_yaw)
        prev_yaw = yaw
        seq += 1

        os.lseek(fd, 0, os.SEEK_SET)
        os.write(fd, struct.pack(FMT, dyaw, DT, 0.0, seq))

        next_t += DT
        sleep = next_t - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_t = time.monotonic()


if __name__ == "__main__":
    main()
