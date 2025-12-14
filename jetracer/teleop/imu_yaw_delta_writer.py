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
MAX_YAW_RATE = 6.0     # rad/s (≈340°/s, 매우 보수적)
MAX_DT = 0.10          # 100 ms 이상은 신뢰 불가

SHM_PATH = "/dev/shm/jetracer_heading_delta"
FMT = "ffI"            # heading_diff(rad), heading_dt(sec), seq
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

    print("[IMU] yaw delta writer (30Hz accumulated, guarded) started")

    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not (line.startswith("#XYMU=") and line.endswith("#")):
            continue

        d = line[6:-1].split(",")   # "#XYMU=" 제거, 끝 "#" 제거
        if len(d) < 7:
            continue

        try:
            qw, qx, qy, qz = map(float, d[3:7])
        except ValueError:
            continue

        yaw = quat_to_yaw(qw, qx, qy, qz)
        now = time.monotonic()

        if prev_yaw is None:
            prev_yaw = yaw
            prev_t = now
            continue

        dt = now - prev_t
        prev_yaw_tmp = prev_yaw  # 디버그용 백업
        prev_yaw = yaw
        prev_t = now

        # ===== dt 방어 =====
        if dt <= 0 or dt > MAX_DT:
            acc_dyaw = 0.0
            acc_dt = 0.0
            continue

        dyaw = wrap(yaw - prev_yaw_tmp)

        # ===== 물리 한계 방어 =====
        if abs(dyaw) > MAX_YAW_RATE * dt:
            # 스파이크 → 누적 리셋
            acc_dyaw = 0.0
            acc_dt = 0.0
            continue

        # ===== 누적 =====
        acc_dyaw += dyaw
        acc_dt += dt

        # ===== 30Hz 윈도우 도달 시에만 emit =====
        if acc_dt >= WINDOW_DT:
            seq += 1  # 🔴 여기서만 seq 증가 (중복 방지)
            os.lseek(fd, 0, os.SEEK_SET)
            os.write(fd, struct.pack(FMT, acc_dyaw, acc_dt, seq))

            acc_dyaw = 0.0
            acc_dt = 0.0


if __name__ == "__main__":
    main()
