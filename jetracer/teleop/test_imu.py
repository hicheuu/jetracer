#!/usr/bin/env python3
from __future__ import annotations

import argparse
import socket
import struct
import time
import serial
import math

from jetracer.teleop.telemetry_common import (
    infer_car_number,
    read_voltage,
)

# =============================
# Config
# =============================
FMT_UPLINK = "!ifffI"
TARGET_HZ = 30.0
WINDOW_DT = 1.0 / TARGET_HZ

MAX_YAW_RATE = 6.0
MAX_DT = 0.05

# =============================
# Quaternion utils
# =============================
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
    qd = quat_mul(quat_conj(q_prev), q_now)
    qw, qx, qy, qz = qd
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny, cosy)

# --- absolute yaw ---
def quat_to_yaw(q):
    qw, qx, qy, qz = q
    return math.atan2(
        2.0 * (qw*qz + qx*qy),
        1.0 - 2.0 * (qy*qy + qz*qz)
    )

def unwrap(d):
    while d > math.pi:
        d -= 2*math.pi
    while d < -math.pi:
        d += 2*math.pi
    return d

# =============================
# Args
# =============================
def build_parser():
    p = argparse.ArgumentParser()
    p.add_argument("--server-ip", required=True)
    p.add_argument("--server-port", type=int, default=5560)
    p.add_argument("--hz", type=float, default=60.0)
    p.add_argument("--car-number", type=int, default=None)
    p.add_argument("--battery-shm-path", default="/dev/shm/jetracer_voltage")
    p.add_argument("--imu-port", default="/dev/ttyACM1")
    p.add_argument("--imu-baud", type=int, default=115200)
    p.add_argument("--poll-sleep", type=float, default=0.0001)
    p.add_argument("--verbose", action="store_true")
    return p

# =============================
# Main
# =============================
def main():
    args = build_parser().parse_args()
    vehicle_id = infer_car_number(args.car_number)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.server_ip, args.server_port)

    ser = serial.Serial(args.imu_port, args.imu_baud, timeout=0)
    time.sleep(1)

    min_interval = 1.0 / max(1.0, float(args.hz))
    next_allowed_send = 0.0

    prev_q = None
    prev_t = None
    prev_yaw = None

    acc_dyaw_A = 0.0   # A: delta quaternion yaw
    acc_dyaw_B = 0.0   # B: absolute yaw + unwrap
    acc_dt = 0.0
    seq = 0

    serial_buffer = b""

    # ---- stats ----
    cnt_total = cnt_used = cnt_skip_dt = cnt_skip_rate = cnt_skip_parse = 0
    last_report_t = time.monotonic()

    print("[uplink] A/B comparison mode started")

    try:
        while True:
            # =============================
            # Serial non-blocking parse
            # =============================
            waiting = ser.in_waiting
            if waiting > 0:
                serial_buffer += ser.read(waiting)
            else:
                time.sleep(args.poll_sleep)
                continue

            if b"\n" not in serial_buffer:
                continue

            parts = serial_buffer.split(b"\n")
            serial_buffer = parts[-1]

            valid = None
            for raw in reversed(parts[:-1]):
                r = raw.strip()
                if r.startswith(b"#XYMU=") and r.endswith(b"#"):
                    valid = r
                    break

            if not valid:
                continue

            line = valid.decode(errors="ignore").strip()

            # =============================
            # Parse
            # =============================
            d = line[6:-1].split(",")
            if len(d) < 7:
                cnt_skip_parse += 1
                continue

            try:
                qw, qx, qy, qz = map(float, d[3:7])
            except ValueError:
                cnt_skip_parse += 1
                continue

            cnt_total += 1
            q_now = (qw, qx, qy, qz)
            now = time.monotonic()

            if prev_q is None:
                prev_q = q_now
                prev_t = now
                prev_yaw = quat_to_yaw(q_now)
                continue

            dt = now - prev_t
            if dt <= 0.0 or dt > MAX_DT:
                cnt_skip_dt += 1
                prev_q = q_now
                prev_t = now
                prev_yaw = quat_to_yaw(q_now)
                continue

            # =============================
            # A: delta quaternion yaw
            # =============================
            dyaw_A = quat_delta_yaw(prev_q, q_now)

            if abs(dyaw_A) > MAX_YAW_RATE * dt:
                cnt_skip_rate += 1
                continue

            # =============================
            # B: absolute yaw + unwrap
            # =============================
            yaw_now = quat_to_yaw(q_now)
            dyaw_B = unwrap(yaw_now - prev_yaw)

            cnt_used += 1
            prev_q = q_now
            prev_t = now
            prev_yaw = yaw_now

            acc_dyaw_A += dyaw_A
            acc_dyaw_B += dyaw_B
            acc_dt += dt

            # =============================
            # Window send
            # =============================
            if acc_dt >= WINDOW_DT:
                seq += 1

                now_send = time.monotonic()
                if now_send >= next_allowed_send:
                    next_allowed_send = now_send + min_interval

                    voltage = read_voltage(args.battery_shm_path) or 0.0
                    pkt = struct.pack(
                        FMT_UPLINK,
                        int(vehicle_id),
                        float(voltage),
                        float(acc_dyaw_A),  # 기존 A 방식 그대로 UDP 송신
                        float(acc_dt),
                        int(seq),
                    )
                    sock.sendto(pkt, target)

                    # ---- 비교 로그 ----
                    print(
                        f"A(delta-q)={acc_dyaw_A:+.4f} rad | "
                        f"B(abs-yaw)={acc_dyaw_B:+.4f} rad | "
                        f"dt={acc_dt:.4f}"
                    )

                acc_dyaw_A = 0.0
                acc_dyaw_B = 0.0
                acc_dt = 0.0

            # =============================
            # 1s stats
            # =============================
            t = time.monotonic()
            if t - last_report_t >= 1.0:
                print(
                    f"[STAT] total={cnt_total} used={cnt_used} "
                    f"skip_dt={cnt_skip_dt} skip_rate={cnt_skip_rate} "
                    f"skip_parse={cnt_skip_parse}"
                )
                last_report_t = t

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        sock.close()

if __name__ == "__main__":
    main()
