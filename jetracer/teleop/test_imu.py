#!/usr/bin/env python3
from __future__ import annotations

import argparse
import socket
import struct
import time
import serial
import math
from typing import List, Optional, Tuple

from jetracer.teleop.telemetry_common import (
    infer_car_number,
    read_voltage,
)

# vehicle_id(int32), voltage(float32), dyaw(float32), dt(float32), seq(uint32)
FMT_UPLINK = "!ifffI"
PKT_SIZE = struct.calcsize(FMT_UPLINK)

# Telemetry window (send dyaw,dt at ~30Hz by default)
TARGET_HZ = 30.0
WINDOW_DT = 1.0 / TARGET_HZ

# Physics guard
MAX_YAW_RATE = 6.0      # rad/s
MAX_DT = 0.10           # (E) "soft" dt threshold used for filtering logic
HARD_RESET_DT = 0.30    # (B) gap threshold to hard-reset baseline

def quat_conj(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    w, x, y, z = q
    return (w, -x, -y, -z)

def quat_mul(
    q1: Tuple[float, float, float, float],
    q2: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
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
    """
    Returns delta yaw (rad) from q_prev -> q_now using quaternion relative rotation.
    """
    qd = quat_mul(quat_conj(q_prev), q_now)
    qw, qx, qy, qz = qd
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny, cosy)

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    p.add_argument("--server-ip", required=True, default="192.168.1.100")
    p.add_argument("--server-port", type=int, default=5560)
    p.add_argument("--hz", type=float, default=60.0)  # max send rate (rate limit)
    p.add_argument("--car-number", type=int, default=None)
    p.add_argument("--battery-shm-path", default="/dev/shm/jetracer_voltage")
    p.add_argument("--imu-port", default="/dev/ttyACM0")
    p.add_argument("--imu-baud", type=int, default=115200)
    # (D) poll-sleep default increased to 0.001 for CPU efficiency
    p.add_argument("--poll-sleep", type=float, default=0.001)
    p.add_argument("--yaw-scale", type=float, default=1.0,
                   help="Optional yaw scale multiplier. Keep 1.0 unless you must calibrate.")
    p.add_argument("--verbose", action="store_true")
    return p

def parse_quat_from_line(line: str) -> Optional[Tuple[float, float, float, float]]:
    """
    Expected format example: '#XYMU=...,qw,qx,qy,qz,...#'
    """
    if not line.startswith("#XYMU=") or not line.endswith("#"):
        return None
    d = line[6:-1].split(",")
    if len(d) < 7:
        return None
    try:
        qw, qx, qy, qz = map(float, d[3:7])
    except ValueError:
        return None
    return (qw, qx, qy, qz)

def main() -> None:
    args = build_parser().parse_args()
    vehicle_id = infer_car_number(args.car_number)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.server_ip, args.server_port)

    try:
        # Non-blocking
        ser = serial.Serial(args.imu_port, args.imu_baud, timeout=0)
    except Exception as e:
        print(f"[ERROR] IMU 연결 실패: {e}")
        return

    time.sleep(1)

    # Send rate limit (upper bound)
    min_interval = 1.0 / max(1.0, float(args.hz))
    next_allowed_send = 0.0

    # Baseline
    prev_q: Optional[Tuple[float, float, float, float]] = None

    # Accumulators
    acc_dyaw = 0.0
    acc_dt = 0.0
    seq = 0

    # Total angle for debugging (rad)
    total_yaw_rad = 0.0

    # Serial chunk buffer
    serial_buffer = b""
    # (C) Serial buffer size limit (128KB)
    MAX_BUFFER_SIZE = 128 * 1024

    # Time bookkeeping: "since last successful chunk read"
    last_success_chunk_t = time.monotonic()

    if args.verbose:
        print("[uplink] started. burst-aware integration + proportional carry-over")
        print(f"[cfg] WINDOW_DT={WINDOW_DT:.6f}s TARGET_HZ={TARGET_HZ:.2f} yaw_scale={args.yaw_scale}")
        print(f"[cfg] MAX_DT={MAX_DT:.3f}s HARD_RESET_DT={HARD_RESET_DT:.3f}s MAX_YAW_RATE={MAX_YAW_RATE:.2f}rad/s")

    try:
        while True:
            # 1) read all available bytes (non-blocking)
            waiting = 0
            try:
                waiting = ser.in_waiting
            except Exception:
                waiting = 0

            if waiting <= 0:
                time.sleep(args.poll_sleep)
                # Still check if we need to force re-baseline due to silence
                if time.monotonic() - last_success_chunk_t > HARD_RESET_DT:
                    if prev_q is not None:
                        prev_q = None
                        # (B) Hard reset accumulator flush
                        acc_dt = 0.0
                        acc_dyaw = 0.0
                    last_success_chunk_t = time.monotonic()
                continue

            try:
                chunk = ser.read(waiting)
                serial_buffer += chunk
                # (C) Serial buffer protection
                if len(serial_buffer) > MAX_BUFFER_SIZE:
                    serial_buffer = serial_buffer[-(MAX_BUFFER_SIZE // 2):]
            except Exception:
                pass

            # 2) split into complete lines
            if b"\n" not in serial_buffer:
                if time.monotonic() - last_success_chunk_t > HARD_RESET_DT:
                    prev_q = None
                    acc_dt = 0.0
                    acc_dyaw = 0.0
                time.sleep(args.poll_sleep)
                continue

            parts = serial_buffer.split(b"\n")
            serial_buffer = parts[-1]  # keep incomplete tail
            raw_lines = parts[:-1]     # complete lines

            # 3) collect all valid packets in this chunk
            q_list: List[Tuple[float, float, float, float]] = []
            for raw in raw_lines:
                r = raw.strip()
                if not (r.startswith(b"#XYMU=") and r.endswith(b"#")):
                    continue
                line = r.decode(errors="ignore").strip()
                q = parse_quat_from_line(line)
                if q is not None:
                    q_list.append(q)

            if not q_list:
                continue

            now = time.monotonic()
            total_chunk_dt = now - last_success_chunk_t
            if total_chunk_dt < 0.0:
                total_chunk_dt = 0.0

            # 4) hard reset if the gap is huge
            hard_reset = (total_chunk_dt > HARD_RESET_DT) or (prev_q is None)
            if hard_reset:
                # (B) Hard reset flush
                acc_dt = 0.0
                acc_dyaw = 0.0
                
                # baseline reset to first packet of new burst
                prev_q = q_list[0]
                last_success_chunk_t = now
                
                # (A) N packets after reset -> N-1 integration steps
                N_steps = max(1, len(q_list) - 1)
                if total_chunk_dt > HARD_RESET_DT:
                    dt_sample = 1.0 / TARGET_HZ
                else:
                    dt_sample = total_chunk_dt / N_steps if total_chunk_dt > 0 else (1.0 / TARGET_HZ)
                start_idx = 1
                
                if args.verbose and total_chunk_dt > HARD_RESET_DT:
                    print(f"[HardReset] gap={total_chunk_dt:.3f}s N={len(q_list)} steps={N_steps}")
            else:
                # (A) Baseline exists, we integrate from previous prev_q to all N packets in chunk
                N_steps = len(q_list)
                dt_sample = total_chunk_dt / max(1, N_steps)
                last_success_chunk_t = now
                start_idx = 0

            # Avoid dt≈0 filters
            if dt_sample < 1e-6:
                dt_sample = 1e-6

            # 5) integrate all packets sequentially
            for i in range(start_idx, len(q_list)):
                q_now = q_list[i]
                if prev_q is None:
                    prev_q = q_now
                    continue

                dyaw = quat_delta_yaw(prev_q, q_now) * float(args.yaw_scale)

                # (E) Apply yaw-rate guard using dt_sample capped at MAX_DT
                dt_for_filter = min(dt_sample, MAX_DT)

                if abs(dyaw) > (MAX_YAW_RATE * dt_for_filter):
                    prev_q = q_now
                    continue

                # Accept & Accumulate
                acc_dyaw += dyaw
                acc_dt += dt_sample
                total_yaw_rad += dyaw
                prev_q = q_now

                # 6) proportional drain & carry-over
                while acc_dt >= WINDOW_DT:
                    now_send = time.monotonic()
                    if now_send < next_allowed_send:
                        break
                    
                    if acc_dt <= 1e-9:
                        break
                        
                    next_allowed_send = now_send + min_interval
                    
                    frac = WINDOW_DT / acc_dt
                    dyaw_to_send = acc_dyaw * frac

                    seq += 1
                    voltage = read_voltage(args.battery_shm_path) or 0.0

                    pkt = struct.pack(
                        FMT_UPLINK,
                        int(vehicle_id),
                        float(voltage),
                        float(dyaw_to_send),
                        float(WINDOW_DT),
                        int(seq),
                    )

                    try:
                        sock.sendto(pkt, target)
                    except Exception:
                        seq -= 1
                        break

                    if args.verbose:
                        print(f"UDP send: dψ={dyaw_to_send:+.6f} rad dt={WINDOW_DT:.6f} s")

                    acc_dt -= WINDOW_DT
                    acc_dyaw -= dyaw_to_send

                    if acc_dt < 1e-9: acc_dt = 0.0
                    if abs(acc_dyaw) < 1e-12: acc_dyaw = 0.0

    except KeyboardInterrupt:
        deg = total_yaw_rad * 180.0 / math.pi
        print("\n" + "=" * 48)
        print("[STOP] IMU telemetry sender terminated by user")
        print(f"Total yaw: {total_yaw_rad:.6f} rad ({deg:.2f} deg)")
        print("=" * 48)

    finally:
        try: ser.close()
        except: pass
        try: sock.close()
        except: pass

if __name__ == "__main__":
    main()