#!/usr/bin/env python3
from __future__ import annotations

import argparse
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

    # NOTE: sender가 writer에 종속되도록 변경했기 때문에,
    # --hz는 "최대 전송률 상한"으로만 사용 (기본 60Hz)
    p.add_argument("--hz", type=float, default=60.0, help="max send rate cap (Hz)")

    p.add_argument("--car-number", type=int, default=None)
    p.add_argument("--battery-shm-path", default="/dev/shm/jetracer_voltage")
    p.add_argument("--heading-shm-path", default="/dev/shm/jetracer_heading_delta")

    # CPU 과점유 방지용 폴링 슬립 (초)
    p.add_argument("--poll-sleep", type=float, default=0.001, help="poll sleep seconds (default: 0.001)")

    p.add_argument("--verbose", action="store_true")
    return p


def main():
    args = build_parser().parse_args()

    vehicle_id = infer_car_number(args.car_number)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.server_ip, args.server_port)

    # 최대 전송률 상한(너무 빨리 도는 것 방지)
    min_interval = 1.0 / max(1.0, float(args.hz))
    next_allowed_send = 0.0

    last_seq = -1

    if args.verbose:
        print(
            f"[uplink] target={target} "
            f"fmt={FMT_UPLINK} size={PKT_SIZE}B vehicle_id={vehicle_id} "
            f"max_hz={args.hz} poll_sleep={args.poll_sleep}"
        )

    try:
        while True:
            # heading delta 먼저 읽어서 "새 데이터인지" 판단
            heading_diff, heading_dt, heading_seq = read_heading_delta(args.heading_shm_path)

            # 데이터 없으면 대기
            if heading_seq is None:
                time.sleep(args.poll_sleep)
                continue

            # ✅ 핵심: seq가 증가했을 때만 처리
            if int(heading_seq) <= last_seq:
                time.sleep(args.poll_sleep)
                continue

            # 전송률 상한 (필요 없으면 hz 크게 두면 됨)
            now = time.monotonic()
            if now < next_allowed_send:
                time.sleep(min(args.poll_sleep, next_allowed_send - now))
                continue
            next_allowed_send = now + min_interval

            last_seq = int(heading_seq)

            # voltage
            voltage = read_voltage(args.battery_shm_path)
            if voltage is None:
                voltage = 0.0

            # heading 값 None 방어
            if heading_diff is None:
                heading_diff = 0.0
            if heading_dt is None:
                heading_dt = 0.0

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
                    f"dψ={float(heading_diff):+.6f} "
                    f"dt={float(heading_dt):.4f} "
                    f"seq={int(heading_seq)}"
                )

    except KeyboardInterrupt:
        if args.verbose:
            print("\n[uplink] interrupted")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
