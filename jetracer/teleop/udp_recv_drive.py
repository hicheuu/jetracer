#!/usr/bin/env python3
import argparse
import math
import socket
import struct
import time

from jetracer.core import NvidiaRacecar

# ==============================
# 기본 실행 설정(원하는 값으로 수정)
# ==============================
CONFIG = {
    "bind_ip": "0.0.0.0",
    "port": 5555,
    "watchdog": 1.0,
    "deg_max": 30.0,
    "center_norm": -0.2,
    "verbose": True,
}
# ==============================

# 송신 포맷: "!ffI" = big-endian, float(throttle 0..1), float(steering_deg -deg_max..+deg_max), uint32(seq)
FMT = "!ffI"
PACKET_SIZE = struct.calcsize(FMT)


def clamp(x, lo=-1.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x


def is_finite(x):
    return (x is not None) and (not math.isnan(x)) and (not math.isinf(x))


def build_parser():
    p = argparse.ArgumentParser(
        description="UDP receiver -> Jetracer driver (with center bias). 옵션을 생략하면 CONFIG 기본값으로 동작합니다."
    )
    p.add_argument("--bind-ip", default=CONFIG["bind_ip"], help=f"수신 바인드 IP (기본: {CONFIG['bind_ip']})")
    p.add_argument("--port", type=int, default=CONFIG["port"], help=f"수신 포트 (기본: {CONFIG['port']})")
    p.add_argument(
        "--watchdog",
        type=float,
        default=CONFIG["watchdog"],
        help=f"워치독 타임아웃 초 (기본: {CONFIG['watchdog']}s)",
    )
    p.add_argument(
        "--deg-max",
        type=float,
        default=CONFIG["deg_max"],
        help=f"송신 각도의 풀스케일 (기본: ±{CONFIG['deg_max']} deg)",
    )
    p.add_argument(
        "--center-norm",
        type=float,
        default=CONFIG["center_norm"],
        help=f"실제 직진이 되는 정규화 스티어 값 (기본: {CONFIG['center_norm']:+.2f})",
    )
    p.add_argument(
        "--verbose",
        action="store_true" if not CONFIG["verbose"] else "store_false",
        help="수신 로그 출력 토글 (기본: CONFIG 설정값)",
    )
    return p


def main():
    parser = build_parser()
    args = parser.parse_args()

    if hasattr(args, "verbose"):
        pass
    else:
        args.verbose = CONFIG["verbose"]

    car = NvidiaRacecar()

    car.steering = args.center_norm
    car.throttle = 0.0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind_ip, args.port))
    sock.settimeout(1.0)

    last_rx_ts = time.time()
    last_seq = None

    print(f"[UDP] listening on {args.bind_ip}:{args.port}  fmt={FMT} ({PACKET_SIZE} bytes)")
    print(f"[MAP] steer_norm = (deg / {args.deg_max}) + center_norm  (center_norm={args.center_norm:+.2f})")
    if args.verbose:
        print(f"[init] steering -> {args.center_norm:+.3f} (center), throttle -> 0.0")

    try:
        while True:
            try:
                data, addr = sock.recvfrom(64)
            except socket.timeout:
                data = None

            if data and len(data) >= PACKET_SIZE:
                throttle, steering_deg, seq = struct.unpack(FMT, data[:PACKET_SIZE])

                if not (is_finite(throttle) and is_finite(steering_deg)):
                    continue

                last_seq = seq if last_seq is None else seq
                last_rx_ts = time.time()

                raw_norm = steering_deg / float(args.deg_max)
                steer_norm = clamp(raw_norm + args.center_norm, -1.0, 1.0)
                thr = clamp(throttle, 0.0, 1.0)

                car.steering = steer_norm
                car.throttle = thr

                if args.verbose:
                    print(
                        f"seq={seq} thr={thr:+.3f} deg={steering_deg:+.1f} "
                        f"raw_norm={raw_norm:+.3f} -> steer_norm={steer_norm:+.3f}"
                    )

            now = time.time()
            if (now - last_rx_ts) > args.watchdog:
                if (car.steering != args.center_norm) or (car.throttle != 0.0):
                    car.steering = args.center_norm
                    car.throttle = 0.0
                    if args.verbose:
                        print(
                            f"[watchdog] no packets > {args.watchdog:.2f}s → "
                            f"steering={args.center_norm:+.3f}, throttle=0.0"
                        )
                last_rx_ts = now

    except KeyboardInterrupt:
        pass
    finally:
        try:
            car.throttle = 0.0
            car.steering = args.center_norm
        except Exception:
            pass
        sock.close()
        print("exiting... motors stopped (centered).")


if __name__ == "__main__":
    main()


