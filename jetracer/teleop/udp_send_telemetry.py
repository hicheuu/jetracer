#!/usr/bin/env python3
"""
UDP telemetry sender for JetRacer (Yaw Delta version).

Sends to server:
1) vehicle_id        (int32)
2) battery voltage   (float32, V)
3) heading_diff      (float32, rad)
4) heading_dt        (float32, sec)
5) heading_seq       (uint32)

Yaw is NOT absolute.
Yaw is Δψ stream written by imu_yaw_delta_writer (30 Hz).
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import time

from jetracer.teleop.telemetry_common import (
    infer_car_number,
    read_voltage,
    read_heading_delta,
)

# =========================
# Packet format
# =========================
FMT_TELEM = "!ifffI"
PKT_SIZE_TELEM = struct.calcsize(FMT_TELEM)


# =========================
# CLI
# =========================
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="UDP telemetry sender (yaw delta)")
    p.add_argument(
        "--server-ip",
        default=os.getenv("JETRACER_SERVER_IP"),
        help="telemetry server IP (or env JETRACER_SERVER_IP)",
    )
    p.add_argument(
        "--server-port",
        type=int,
        default=int(os.getenv("JETRACER_SERVER_PORT", "5560")),
        help="telemetry server port (default: 5560)",
    )
    p.add_argument("--hz", type=float, default=30.0, help="send rate in Hz (default: 2.0)")
    p.add_argument(
        "--car-number",
        type=int,
        default=None,
        help="override car number (default: parse from username)",
    )
    p.add_argument(
        "--battery-shm-path",
        default="/dev/shm/jetracer_voltage",
        help="battery voltage shm path",
    )
    p.add_argument(
        "--heading-shm-path",
        default="/dev/shm/jetracer_heading_delta",
        help="heading delta shm path",
    )
    p.add_argument("--verbose", action="store_true", help="print sent values")
    return p


# =========================
# Main
# =========================
def main():
    parser = build_parser()
    args = parser.parse_args()

    if not args.server_ip:
        parser.error("--server-ip is required (or set JETRACER_SERVER_IP)")

    vehicle_id = infer_car_number(args.car_number)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.server_ip, args.server_port)

    interval = 1.0 / max(0.1, args.hz)
    next_ts = time.monotonic()

    if args.verbose:
        print(
            f"[telemetry] target={target} fmt={FMT_TELEM} "
            f"({PKT_SIZE_TELEM}B) vehicle_id={vehicle_id}"
        )

    try:
        while True:
            # -------------------------
            # Read battery voltage
            # -------------------------
            voltage = read_voltage(args.battery_shm_path)
            if voltage is None:
                voltage = -1.0

            # -------------------------
            # Read yaw delta
            # -------------------------
            heading_diff, heading_dt, heading_seq = read_heading_delta(
                args.heading_shm_path
            )

            if heading_diff is None:
                heading_diff = 0.0
                heading_dt = 0.0
                heading_seq = 0

            # -------------------------
            # Pack & send
            # -------------------------
            pkt = struct.pack(
                FMT_TELEM,
                int(vehicle_id),
                float(voltage),
                float(heading_diff),
                float(heading_dt),
                int(heading_seq),
            )
            sock.sendto(pkt, target)

            if args.verbose:
                print(
                    f"[telemetry] car={vehicle_id} "
                    f"V={voltage:.2f} "
                    f"dψ={heading_diff:+.5f}rad "
                    f"dt={heading_dt:.4f}s "
                    f"seq={heading_seq}"
                )

            # -------------------------
            # Rate control
            # -------------------------
            next_ts += interval
            sleep_for = next_ts - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_ts = time.monotonic()

    except KeyboardInterrupt:
        if args.verbose:
            print("\n[telemetry] interrupted")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
