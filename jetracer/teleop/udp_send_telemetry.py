#!/usr/bin/env python3
"""
UDP telemetry sender for JetRacer.

Sends 3 fields to server:
1) car number: parsed from username (e.g., jet3 -> 3) or overridden by --car-number
2) battery percentage: derived from /dev/shm/jetracer_voltage (battery_monitor service)
3) Q-yaw (heading): read from /dev/shm/jetracer_qyaw (keyboard_yaw_hold)

Packet format (network byte order): "!Iff"
  uint32 car_number, float32 battery_pct, float32 qyaw_deg

This script does NOT open I2C or serial devices.
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import time

from jetracer.teleop.telemetry_common import (
    infer_car_number,
    read_battery_pct,
    read_qyaw,
    read_voltage,
)


FMT_TELEM = "!Iff"
PKT_SIZE_TELEM = struct.calcsize(FMT_TELEM)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="UDP telemetry sender (car#, battery%, Q-yaw)")
    p.add_argument(
        "--server-ip",
        default=os.getenv("JETRACER_SERVER_IP"),
        help="telemetry server IP (or env JETRACER_SERVER_IP)",
    )
    p.add_argument(
        "--server-port",
        type=int,
        default=int(os.getenv("JETRACER_SERVER_PORT", "5560")),
        help="telemetry server port (default: 5560, or env JETRACER_SERVER_PORT)",
    )
    p.add_argument("--hz", type=float, default=2.0, help="send rate in Hz (default: 2.0)")
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
        "--battery-cells",
        type=int,
        default=2,
        help="battery pack cell count for SOC (default: 2)",
    )
    p.add_argument(
        "--yaw-shm-path",
        default="/dev/shm/jetracer_qyaw",
        help="Q-yaw shm path",
    )
    p.add_argument("--verbose", action="store_true", help="print sent values")
    return p


def main():
    parser = build_parser()
    args = parser.parse_args()

    if not args.server_ip:
        parser.error("--server-ip is required (or set JETRACER_SERVER_IP)")

    car_number = infer_car_number(args.car_number)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.server_ip, args.server_port)

    interval = 1.0 / max(0.1, args.hz)
    next_ts = time.monotonic()

    if args.verbose:
        print(f"[telemetry] target={target} fmt={FMT_TELEM} ({PKT_SIZE_TELEM}B) car_number={car_number}")

    try:
        while True:
            bat_pct = read_battery_pct(args.battery_shm_path, args.battery_cells)
            qyaw = read_qyaw(args.yaw_shm_path)

            if bat_pct is None:
                bat_pct = -1.0
            if qyaw is None:
                qyaw = -1.0

            pkt = struct.pack(FMT_TELEM, int(car_number), float(bat_pct), float(qyaw))
            sock.sendto(pkt, target)

            if args.verbose:
                v = read_voltage(args.battery_shm_path)
                v_disp = v if v is not None else -1.0
                print(f"[telemetry] car={car_number} bat={bat_pct:.1f}% v={v_disp:.2f} qyaw={qyaw:.1f}")

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

