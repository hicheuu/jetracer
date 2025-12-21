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

FMT_UPLINK = "!ifffI"
PKT_SIZE = struct.calcsize(FMT_UPLINK)

TARGET_HZ = 30.0
WINDOW_DT = 1.0 / TARGET_HZ

MAX_YAW_RATE = 6.0
MAX_DT = 0.05

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
    return math.atan2(siny, cosy)  # rad

def build_parser():
    p = argparse.ArgumentParser()
    p.add_argument("--server-ip", required=True)
    p.add_argument("--server-port", type=int, default=5560)
    p.add_argument("--hz", type=float, default=60.0)
    p.add_argument("--car-number", type=int, default=None)
    p.add_argument("--battery-shm-path", default="/dev/shm/jetracer_voltage")
    p.add_argument("--imu-port", default="/dev/ttyACM0")
    p.add_argument("--imu-baud", type=int, default=115200)
    p.add_argument("--poll-sleep", type=float, default=0.0001)
    p.add_argument("--verbose", action="store_true")
    return p

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
    acc_dyaw = 0.0
    acc_dt = 0.0
    seq = 0

    serial_buffer = b""

    total_yaw_rad = 0.0  # [ADDED] 종료 시 총 누적 yaw(rad)

    if args.verbose:
        print(f"[uplink] started. raw-parsing mode.")

    try:
        while True:
            try:
                waiting = ser.in_waiting
                if waiting > 0:
                    chunk = ser.read(waiting)
                    serial_buffer += chunk
                else:
                    time.sleep(args.poll_sleep)
                    continue

                if b'\n' in serial_buffer:
                    parts = serial_buffer.split(b'\n')
                    serial_buffer = parts[-1]

                    valid_line = None
                    for raw_line in reversed(parts[:-1]):
                        r = raw_line.strip()
                        if r.startswith(b"#XYMU=") and r.endswith(b"#"):
                            valid_line = r
                            break

                    if valid_line:
                        line = valid_line.decode(errors="ignore").strip()
                    else:
                        continue
                else:
                    continue

            except OSError:
                time.sleep(0.1)
                continue
            except Exception as e:
                if args.verbose:
                    print(f"Parse Error: {e}")
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

            if prev_q is None:
                prev_q = q_now
                prev_t = now
                continue

            dt = now - prev_t

            if dt <= 0.0 or dt > MAX_DT:
                prev_q = q_now
                prev_t = now
                continue

            dyaw = quat_delta_yaw(prev_q, q_now)

            if abs(dyaw) > MAX_YAW_RATE * dt:
                continue

            prev_q = q_now
            prev_t = now

            acc_dyaw += dyaw
            acc_dt += dt
            total_yaw_rad += dyaw  # [ADDED] 누적

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
                        float(acc_dyaw),
                        float(acc_dt),
                        int(seq),
                    )
                    sock.sendto(pkt, target)

                    if args.verbose:
                        print(f"UDP send: dψ={acc_dyaw:+.4f} rad dt={acc_dt:.4f}")

                acc_dyaw = 0.0
                acc_dt = 0.0

    except KeyboardInterrupt:
        # [ADDED] 종료 시 총 각도 출력 (rad -> deg)
        total_yaw_deg = total_yaw_rad * (180.0 / math.pi)
        print("\n" + "=" * 40)
        print("🛑 종료")
        print(f"총 yaw 변화량: {total_yaw_rad:+.6f} rad")
        print(f"총 yaw 변화량: {total_yaw_deg:+.2f} deg")
        print("=" * 40)

    finally:
        ser.close()
        sock.close()

if __name__ == "__main__":
    main()
