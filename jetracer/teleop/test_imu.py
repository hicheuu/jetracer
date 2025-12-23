#!/usr/bin/env python3
from __future__ import annotations

import argparse
import socket
import struct
import time
import serial
import math
from typing import Optional, Tuple

from jetracer.teleop.telemetry_common import infer_car_number, read_voltage

# vehicle_id(int32), voltage(float32), dyaw(float32), dt(float32), seq(uint32)
FMT_UPLINK = "!ifffI"

MAX_YAW_RATE = 6.0       # rad/s
HARD_RESET_DT = 0.30     # silence/gap threshold

def quat_dot(q1: Tuple[float, float, float, float],
             q2: Tuple[float, float, float, float]) -> float:
    return q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]

def quat_normalize(q: Tuple[float, float, float, float]) -> Optional[Tuple[float, float, float, float]]:
    n = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
    if not (0.8 <= n <= 1.2):
        return None
    return (q[0]/n, q[1]/n, q[2]/n, q[3]/n)

def quat_conj(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    return (q[0], -q[1], -q[2], -q[3])

def quat_mul(q1: Tuple[float, float, float, float],
             q2: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )

def quat_delta_yaw(q_prev: Tuple[float, float, float, float],
                   q_now: Tuple[float, float, float, float]) -> float:
    qd = quat_mul(quat_conj(q_prev), q_now)
    qw, qx, qy, qz = qd
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny, cosy)

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    p.add_argument("--server-ip", required=True)
    p.add_argument("--server-port", type=int, default=5560)
    p.add_argument("--car-number", type=int, default=None)
    p.add_argument("--battery-shm-path", default="/dev/shm/jetracer_voltage")
    p.add_argument("--imu-hz", type=float, default=100.0, help="Nominal IMU output rate")
    p.add_argument("--imu-port", default="/dev/ttyACM0")
    p.add_argument("--imu-baud", type=int, default=115200)
    p.add_argument("--poll-sleep", type=float, default=0.001)
    p.add_argument("--yaw-scale", type=float, default=1.0)
    p.add_argument("--verbose", action="store_true")
    return p

def main() -> None:
    args = build_parser().parse_args()

    vehicle_id = infer_car_number(args.car_number)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.server_ip, args.server_port)

    try:
        ser = serial.Serial(args.imu_port, args.imu_baud, timeout=0)
    except Exception as e:
        print(f"[ERROR] IMU Connection Failed: {e}")
        return

    # nominal dt
    sub_dt_nom = 1.0 / max(1.0, float(args.imu_hz))

    # Buffer management
    serial_buffer = bytearray()
    MAX_BUFFER_SIZE = 128 * 1024

    prev_q: Optional[Tuple[float, float, float, float]] = None
    last_success_t = time.monotonic()

    seq = 0
    total_yaw = 0.0

    # consecutive spike counter (for self-healing)
    spike_streak = 0
    SPIKE_STREAK_RESET = 8

    print(f"[Uplink] Robust Integration Mode (imu_hz={args.imu_hz:.1f}Hz) -> {target[0]}:{target[1]}")

    try:
        while True:
            # ---- non-blocking read ----
            try:
                waiting = ser.in_waiting
                if waiting > 0:
                    serial_buffer.extend(ser.read(waiting))
            except Exception:
                pass

            # buffer overflow protection
            if len(serial_buffer) > MAX_BUFFER_SIZE:
                del serial_buffer[:len(serial_buffer) - (MAX_BUFFER_SIZE // 2)]

            # no full line yet
            if b"\n" not in serial_buffer:
                if time.monotonic() - last_success_t > HARD_RESET_DT:
                    prev_q = None
                    spike_streak = 0
                time.sleep(args.poll_sleep)
                continue

            parts = serial_buffer.split(b"\n")
            serial_buffer = bytearray(parts[-1])

            # parse all valid quaternions
            q_list: list[Tuple[float, float, float, float]] = []
            for raw in parts[:-1]:
                line = raw.strip()
                if not (line.startswith(b"#XYMU=") and line.endswith(b"#")):
                    continue
                try:
                    d = line[6:-1].split(b",")
                    if len(d) < 7:
                        continue
                    q_raw = (float(d[3]), float(d[4]), float(d[5]), float(d[6]))  # w,x,y,z
                except Exception:
                    continue

                qn = quat_normalize(q_raw)
                if qn is not None:
                    q_list.append(qn)

            if not q_list:
                continue

            now = time.monotonic()
            real_gap = now - last_success_t
            last_success_t = now

            # hard reset baseline on long silence
            if prev_q is None or real_gap > HARD_RESET_DT:
                prev_q = q_list[0]
                spike_streak = 0
                if len(q_list) == 1:
                    continue
                integration_list = q_list[1:]
            else:
                integration_list = q_list

            # --- dt_eff for clamps: never smaller than nominal, never absurdly larger ---
            # NOTE: this uses host time only to relax (increase) dt, not to shrink it.
            dt_eff = max(sub_dt_nom, real_gap / max(1, len(integration_list)))
            dt_eff = min(dt_eff, 5.0 * sub_dt_nom)  # cap: avoid over-trusting scheduler stalls

            limit = MAX_YAW_RATE * dt_eff
            spike_threshold = 5.0 * limit

            burst_dyaw = 0.0
            burst_dt_sum = 0.0

            for q_now in integration_list:
                assert prev_q is not None

                # sign flip continuity
                if quat_dot(prev_q, q_now) < 0.0:
                    q_now = (-q_now[0], -q_now[1], -q_now[2], -q_now[3])

                dy = quat_delta_yaw(prev_q, q_now) * float(args.yaw_scale)

                # spike drop: do NOT overwrite baseline with possibly corrupt q_now
                if abs(dy) > spike_threshold:
                    spike_streak += 1
                    if spike_streak >= SPIKE_STREAK_RESET:
                        prev_q = None  # force re-baseline
                        spike_streak = 0
                        break
                    continue

                spike_streak = 0

                # mild clamp
                dy = max(-limit, min(dy, limit))

                burst_dyaw += dy
                burst_dt_sum += dt_eff
                total_yaw += dy

                prev_q = q_now

            if burst_dt_sum <= 0.0:
                continue

            seq += 1
            voltage = read_voltage(args.battery_shm_path) or 0.0

            pkt = struct.pack(
                FMT_UPLINK,
                int(vehicle_id),
                float(voltage),
                float(burst_dyaw),
                float(burst_dt_sum),
                int(seq),
            )

            try:
                sock.sendto(pkt, target)
            except Exception:
                pass

            if args.verbose:
                print(f"[Burst] N={len(q_list)} dy={burst_dyaw:+.6f} rad dt={burst_dt_sum:.6f} (dt_eff={dt_eff:.6f})")

    except KeyboardInterrupt:
        print(f"\n[STOP] Total yaw = {total_yaw:.6f} rad ({math.degrees(total_yaw):.2f} deg)")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        try:
            sock.close()
        except Exception:
            pass
 
if __name__ == "__main__":
    main()

