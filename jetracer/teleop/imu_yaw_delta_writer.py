#!/usr/bin/env python3
import serial
import math
import time
import struct
import os

PORT = "/dev/ttyACM0"
BAUD = 115200
HZ = 30.0
DT = 1.0 / HZ

SHM_PATH = "/dev/shm/jetracer_heading_delta"
FMT = "fffI"  # heading_diff, heading_dt, dummy, seq
SIZE = struct.calcsize(FMT)


def quat_to_yaw(qw, qx, qy, qz):
    siny = 2.0 * (qw*qz + qx*qy)
    cosy = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny, cosy)


def wrap(a):
    if a > math.pi:
        a -= 2 * math.pi
    elif a < -math.pi:
        a += 2 * math.pi
    return a


def main():
    # timeout 짧게 (블로킹 최소화)
    ser = serial.Serial(PORT, BAUD, timeout=0.02)
    time.sleep(1)

    fd = os.open(SHM_PATH, os.O_CREAT | os.O_RDWR)
    os.ftruncate(fd, SIZE)

    prev_yaw = None
    seq = 0

    while True:
        loop_start = time.monotonic()

        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("#XYMU="):
            d = line.split("=")[1].split(",")
            if len(d) >= 7:
                qw, qx, qy, qz = map(float, d[3:7])
                yaw = quat_to_yaw(qw, qx, qy, qz)

                if prev_yaw is not None:
                    dyaw = wrap(yaw - prev_yaw)
                    seq += 1

                    os.lseek(fd, 0, os.SEEK_SET)
                    os.write(fd, struct.pack(FMT, dyaw, DT, 0.0, seq))

                prev_yaw = yaw

        # ---- 실시간-safe sleep ----
        elapsed = time.monotonic() - loop_start
        sleep_time = DT - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
        # sleep_time <= 0 이면 그냥 다음 루프로 (누적 ❌)


if __name__ == "__main__":
    main()
