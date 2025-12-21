#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
XYMU serial reader (NO ROS) + A/B/C yaw comparison + optional UDP uplink.

Input line example:
  ... #XYMU=ax,ay,az,qw,qx,qy,qz,gx,gy,gz# ...

Index meaning (0-based) matches your ROS code:
  [0..2] accel xyz
  [3..6] quat wxyz
  [7..9] gyro xyz  (assumed rad/s)

Outputs:
  A(delta-q)  : delta yaw from dq = conj(q_prev) * q_curr
  B(abs-yaw)  : yaw from current quaternion
  C(gyro)     : dpsi = gz*dt, yaw_gyro accumulated

UDP payload (default):
  struct "!ffI" => (dpsi, dt, seq)
"""

import argparse
import math
import re
import signal
import socket
import struct
import sys
import time

import serial


# ----------------------------
# Math helpers
# ----------------------------
def wrap_pi(x: float) -> float:
    # [-pi, pi)
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
    # yaw (Z-axis rotation) from quaternion (w,x,y,z)
    w, x, y, z = q
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


# ----------------------------
# XYMU parsing
# ----------------------------
XYMU_RE = re.compile(r"#\s*XYMU\s*=\s*([^#]+)#")


def parse_xymu(line_bytes):
    """
    Return list of 10 floats or None
    """
    if not line_bytes:
        return None

    try:
        s = line_bytes.decode("utf-8", "ignore") if isinstance(line_bytes, (bytes, bytearray)) else str(line_bytes)
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
        vals = [float(parts[i]) for i in range(10)]
        return vals
    except Exception:
        return None


# ----------------------------
# Main
# ----------------------------
def build_parser():
    p = argparse.ArgumentParser(description="XYMU IMU uplink (NO ROS)")

    p.add_argument("--port", default="/dev/ttyACM0", help="Serial port (e.g., /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=115200, help="Serial baudrate")
    p.add_argument("--flush", type=int, default=30, help="Flush first N lines after open")
    p.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout (sec)")

    p.add_argument("--target-hz", type=float, default=30.0, help="Only used for log expectation; loop is driven by serial")
    p.add_argument("--max-dt", type=float, default=0.10, help="Skip sample if dt > max-dt (sec)")
    p.add_argument("--max-yaw-rate", type=float, default=6.0, help="Clamp/skip if |gz| > max-yaw-rate (rad/s)")

    p.add_argument("--invert-gz", action="store_true", help="Invert gyro z sign")
    p.add_argument("--invert-quat-yaw", action="store_true", help="Invert quaternion-derived yaw sign")

    # UDP
    p.add_argument("--udp-ip", default="", help="If set, send UDP packets")
    p.add_argument("--udp-port", type=int, default=0, help="UDP port")
    p.add_argument("--send-mode", choices=["gyro", "deltaq"], default="gyro",
                   help="Which dpsi to send via UDP")
    p.add_argument("--udp-fmt", choices=["ffI", "ffffI"], default="ffI",
                   help="Payload format: ffI=(dpsi,dt,seq), ffffI=(dpsi,dt,yaw_gyro,yaw_abs,seq)")

    # Logging
    p.add_argument("--print-every", type=int, default=1, help="Print every N samples (1 = print all)")
    return p


STOP = False


def on_sigint(signum, frame):
    global STOP
    STOP = True


def main():
    global STOP
    args = build_parser().parse_args()

    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)

    # UDP socket (optional)
    sock = None
    dst = None
    if args.udp_ip and args.udp_port > 0:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dst = (args.udp_ip, args.udp_port)

    # Open serial
    print(f"[INFO] Opening serial: {args.port} @ {args.baud}")
    print("[INFO] Waiting 3 seconds for IMU boot...")
    time.sleep(3.0)

    try:
        ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=args.timeout)
    except Exception as e:
        print(f"[ERROR] Failed to open serial {args.port}: {e}")
        return 1

    # Flush
    print(f"[INFO] Flushing first {args.flush} lines...")
    for _ in range(max(0, args.flush)):
        try:
            ser.readline()
        except Exception:
            pass

    # State
    prev_t = None
    prev_q = None
    prev_yaw_abs = None

    yaw_gyro = 0.0
    seq = 0

    # Stats
    total = used = skip_dt = skip_rate = skip_parse = 0
    stat_every = 72  # roughly 30 Hz -> ~2.4 sec

    print("[INFO] Running... Ctrl+C to stop.")
    print("[INFO] Columns: A(delta-q) | B(abs-yaw) | C(gyro dpsi, yaw_gyro) | dt")

    while not STOP:
        line = ser.readline()
        total += 1

        raw = parse_xymu(line)
        if raw is None:
            skip_parse += 1
            continue

        # Extract
        ax, ay, az = raw[0], raw[1], raw[2]
        qw, qx, qy, qz = raw[3], raw[4], raw[5], raw[6]
        gx, gy, gz = raw[7], raw[8], raw[9]

        if args.invert_gz:
            gz = -gz

        # dt
        now = time.time()
        if prev_t is None:
            prev_t = now
            # 첫 프레임은 기준만 잡고 넘어감
            continue

        dt = now - prev_t
        prev_t = now

        if dt <= 0.0 or dt > args.max_dt:
            skip_dt += 1
            continue

        # quaternion
        q = (qw, qx, qy, qz)

        # B: abs yaw from quaternion
        yaw_abs = yaw_from_quat_wxyz(q)
        if args.invert_quat_yaw:
            yaw_abs = -yaw_abs

        # A: delta-q yaw
        dpsi_deltaq = 0.0
        if prev_q is not None:
            dq = quat_mul(quat_conj(prev_q), q)
            dpsi_deltaq = wrap_pi(yaw_from_quat_wxyz(dq))
            if args.invert_quat_yaw:
                dpsi_deltaq = -dpsi_deltaq
        prev_q = q

        # C: gyro integration
        # spike guard
        if abs(gz) > args.max_yaw_rate:
            skip_rate += 1
            dpsi_gyro = 0.0
        else:
            dpsi_gyro = gz * dt
            yaw_gyro = wrap_pi(yaw_gyro + dpsi_gyro)

        # Also (optional) delta from abs yaw
        if prev_yaw_abs is None:
            dpsi_abs = 0.0
        else:
            dpsi_abs = wrap_pi(yaw_abs - prev_yaw_abs)
        prev_yaw_abs = yaw_abs

        used += 1
        seq += 1

        # Print
        if args.print_every > 0 and (seq % args.print_every == 0):
            print(
                f"A(delta-q)={dpsi_deltaq:+.4f} rad | "
                f"B(abs-yaw)={yaw_abs:+.4f} rad (dAbs={dpsi_abs:+.4f}) | "
                f"C(gyro dpsi)={dpsi_gyro:+.4f} rad yaw_gyro={yaw_gyro:+.4f} | "
                f"dt={dt:.4f}"
            )

        # UDP send
        if sock is not None:
            dpsi_send = dpsi_gyro if args.send_mode == "gyro" else dpsi_deltaq

            if args.udp_fmt == "ffI":
                pkt = struct.pack("!ffI", float(dpsi_send), float(dt), int(seq))
            else:
                pkt = struct.pack("!ffffI", float(dpsi_send), float(dt), float(yaw_gyro), float(yaw_abs), int(seq))

            try:
                sock.sendto(pkt, dst)
            except Exception as e:
                print(f"[WARN] UDP send failed: {e}")

        # Periodic stats
        if used > 0 and (used % stat_every == 0):
            print(f"[STAT] total={total} used={used} skip_dt={skip_dt} skip_rate={skip_rate} skip_parse={skip_parse}")

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
