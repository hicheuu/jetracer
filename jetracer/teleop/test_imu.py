#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import re
import signal
import socket
import struct
import time
from collections import deque
from statistics import median

import serial


XYMU_RE = re.compile(r"#\s*XYMU\s*=\s*([^#]+)#")

STOP = False


def on_sig(signum, frame):
    global STOP
    STOP = True


def wrap_pi(x: float) -> float:
    while x >= math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


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


def yaw_from_quat_wxyz(q):
    w, x, y, z = q
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def parse_xymu(line_bytes):
    if not line_bytes:
        return None
    try:
        s = line_bytes.decode("utf-8", "ignore")
    except Exception:
        s = str(line_bytes)

    m = XYMU_RE.search(s)
    if not m:
        return None

    payload = m.group(1).strip()
    parts = [p.strip() for p in payload.split(",")]
    if len(parts) < 10:
        return None

    try:
        return [float(parts[i]) for i in range(10)]
    except Exception:
        return None


def build_parser():
    p = argparse.ArgumentParser(description="XYMU reader (NO ROS) v2: dt-stable + deltaq order select")

    p.add_argument("--port", default="/dev/ttyACM1")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--timeout", type=float, default=1.0)
    p.add_argument("--flush", type=int, default=30)

    p.add_argument("--max-dt", type=float, default=0.20, help="Skip if dt_obs > max-dt (sec)")
    p.add_argument("--dt-window", type=int, default=51, help="Median dt window size (odd recommended)")
    p.add_argument("--min-dt", type=float, default=0.002, help="Ignore dt_obs smaller than this (sec)")

    p.add_argument("--gyro-unit", choices=["rad", "deg"], default="rad", help="Unit of gx/gy/gz")
    p.add_argument("--max-yaw-rate", type=float, default=12.0, help="rad/s after conversion")

    p.add_argument("--print-every", type=int, default=1)

    # UDP (optional)
    p.add_argument("--udp-ip", default="")
    p.add_argument("--udp-port", type=int, default=0)
    p.add_argument("--send-mode", choices=["gyro", "deltaq"], default="gyro")
    p.add_argument("--udp-fmt", choices=["ffI", "ffffI"], default="ffI")

    return p


def main():
    args = build_parser().parse_args()

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    sock = None
    dst = None
    if args.udp_ip and args.udp_port > 0:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dst = (args.udp_ip, args.udp_port)

    print(f"[INFO] Opening serial: {args.port} @ {args.baud}")
    print("[INFO] Waiting 3 seconds for IMU boot...")
    time.sleep(3.0)

    try:
        ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=args.timeout)
    except Exception as e:
        print(f"[ERROR] open serial failed: {e}")
        return 1

    print(f"[INFO] Flushing first {args.flush} lines...")
    for _ in range(max(0, args.flush)):
        try:
            ser.readline()
        except Exception:
            pass

    # state
    prev_t = None
    prev_q = None
    prev_yaw_abs = None

    yaw_gyro = 0.0
    seq = 0

    dt_hist = deque(maxlen=max(5, args.dt_window))
    dt_est = None  # median(dt_hist)

    total = used = skip_parse = skip_dt = skip_rate = 0

    print("[INFO] Running... Ctrl+C to stop.")
    print("[INFO] Columns: A(delta-q) | B(abs-yaw) | C(gyro) | dt_obs/dt_est | gz")

    while not STOP:
        line = ser.readline()
        total += 1

        raw = parse_xymu(line)
        if raw is None:
            skip_parse += 1
            continue

        ax, ay, az = raw[0], raw[1], raw[2]
        qw, qx, qy, qz = raw[3], raw[4], raw[5], raw[6]
        gx, gy, gz = raw[7], raw[8], raw[9]

        # time
        now = time.monotonic()
        if prev_t is None:
            prev_t = now
            prev_q = (qw, qx, qy, qz)
            prev_yaw_abs = yaw_from_quat_wxyz(prev_q)
            continue

        dt_obs = now - prev_t
        prev_t = now

        if dt_obs > args.max_dt:
            skip_dt += 1
            continue
        if dt_obs >= args.min_dt:
            dt_hist.append(dt_obs)
        if len(dt_hist) >= 5:
            dt_est = median(dt_hist)
        else:
            dt_est = dt_obs

        # gyro unit -> rad/s
        if args.gyro_unit == "deg":
            gz_rad = gz * (math.pi / 180.0)
        else:
            gz_rad = gz

        if abs(gz_rad) > args.max_yaw_rate:
            skip_rate += 1
            gz_rad_use = 0.0
        else:
            gz_rad_use = gz_rad

        # quaternion
        q = (qw, qx, qy, qz)
        yaw_abs = yaw_from_quat_wxyz(q)

        dAbs = 0.0 if prev_yaw_abs is None else wrap_pi(yaw_abs - prev_yaw_abs)
        prev_yaw_abs = yaw_abs

        # delta-q: try both orders
        dpsi_a = 0.0
        dpsi_b = 0.0
        if prev_q is not None:
            dq_a = quat_mul(quat_conj(prev_q), q)  # prev^{-1} * curr
            dq_b = quat_mul(q, quat_conj(prev_q))  # curr * prev^{-1}
            dpsi_a = wrap_pi(yaw_from_quat_wxyz(dq_a))
            dpsi_b = wrap_pi(yaw_from_quat_wxyz(dq_b))

        # choose delta-q that best matches dAbs (when dAbs is meaningful)
        if abs(dAbs) > 1e-4:
            err_a = abs(wrap_pi(dpsi_a - dAbs))
            err_b = abs(wrap_pi(dpsi_b - dAbs))
            dpsi_deltaq = dpsi_a if err_a <= err_b else dpsi_b
        else:
            # when abs yaw didn't change, keep small one (usually 0)
            dpsi_deltaq = dpsi_a if abs(dpsi_a) <= abs(dpsi_b) else dpsi_b

        prev_q = q

        # gyro integration using dt_est (stable)
        dpsi_gyro = gz_rad_use * dt_est
        yaw_gyro = wrap_pi(yaw_gyro + dpsi_gyro)

        used += 1
        seq += 1

        if args.print_every > 0 and (seq % args.print_every == 0):
            hz_est = (1.0 / dt_est) if dt_est and dt_est > 0 else 0.0
            print(
                f"A(delta-q)={dpsi_deltaq:+.4f} rad | "
                f"B(abs-yaw)={yaw_abs:+.4f} rad (dAbs={dAbs:+.4f}) | "
                f"C(gyro dpsi)={dpsi_gyro:+.4f} rad yaw_gyro={yaw_gyro:+.4f} | "
                f"dt_obs={dt_obs:.4f} dt_est={dt_est:.4f} (~{hz_est:.1f}Hz) | "
                f"gz={gz_rad_use:+.4f} rad/s"
            )

        # UDP send
        if sock is not None:
            dpsi_send = dpsi_gyro if args.send_mode == "gyro" else dpsi_deltaq
            if args.udp_fmt == "ffI":
                pkt = struct.pack("!ffI", float(dpsi_send), float(dt_est), int(seq))
            else:
                pkt = struct.pack("!ffffI", float(dpsi_send), float(dt_est), float(yaw_gyro), float(yaw_abs), int(seq))
            try:
                sock.sendto(pkt, dst)
            except Exception as e:
                print(f"[WARN] UDP send failed: {e}")

        if used > 0 and (used % 200 == 0):
            print(f"[STAT] total={total} used={used} skip_parse={skip_parse} skip_dt={skip_dt} skip_rate={skip_rate}")

    print("\n[INFO] Exiting...")
    try:
        ser.close()
    except Exception:
        pass
    if sock is not None:
        try:
            sock.close()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
