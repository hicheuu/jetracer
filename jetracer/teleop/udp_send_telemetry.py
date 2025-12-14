#!/usr/bin/env python3
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
# Binary packet definition
# =========================
FMT_UPLINK = "!ifffI"
PKT_SIZE = struct.calcsize(FMT_UPLINK)  # 20 bytes


def build_parser():
    p = argparse.ArgumentParser(description="UDP telemetry sender (binary uplink)")
    p.add_argument("--server-ip", required=True, help="server IP")
    p.add_argument("--server-port", type=int, default=5560, help="server port")
    p.add_argument("--hz", type=float, default=30.0, help="send rate (Hz)")
    p.add_argument("--car-number", type=int, default=None)
    p.add_argument("--battery-shm-path", default="/dev/shm/jetracer_voltage")
    p.add_argument("--heading-shm-path", default="/dev/shm/jetracer_heading_delta")
    p.add_argument("--verbose", action="store_true")
    return p


def main():
    args = build_parser().parse_args()

    vehicle_id = infer_car_number(args.car_number)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.server_ip, args.server_port)

    interval = 1.0 / max(0.1, args.hz)
    next_ts = time.monotonic()

    if args.verbose:
        print(
            f"[uplink] target={target} "
            f"fmt={FMT_UPLINK} size={PKT_SIZE}B vehicle_id={vehicle_id}"
        )

    try:
        while True:
            # -------------------------
            # voltage
            # -------------------------
            voltage = read_voltage(args.battery_shm_path)
            if voltage is None:
                voltage = 0.0

            # -------------------------
            # heading delta
            # -------------------------
            heading_diff, heading_dt, heading_seq = read_heading_delta(
                args.heading_shm_path
            )

            if heading_diff is None:
                heading_diff = 0.0
                heading_dt = 0.0
                heading_seq = 0

            # -------------------------
            # pack & send (THE CORE)
            # -------------------------
            pkt = struct.pack(
                FMT_UPLINK,
                int(vehicle_id),
                float(voltage),
                float(heading_diff),
                float(heading_dt),
                int(heading_seq),
            )
            sock.sendto(pkt, target)

            if args.verbose:
                print(
                    f"[uplink] vid={vehicle_id} "
                    f"V={voltage:.2f} "
                    f"dψ={heading_diff:+.6f} "
                    f"dt={heading_dt:.4f} "
                    f"seq={heading_seq}"
                )

            next_ts += interval
            sleep_for = next_ts - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_ts = time.monotonic()

    except KeyboardInterrupt:
        if args.verbose:
            print("\n[uplink] interrupted")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
