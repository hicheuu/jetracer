#!/usr/bin/env python3
import socket
import struct
import time
import math
import argparse

from jetracer.nvidia_racecar import NvidiaRacecar

# ==============================
# 기본 실행 설정(원하는 값으로 수정)
# ==============================
CONFIG = {
    "bind_ip": "0.0.0.0",   # 수신 바인드 IP
    "port": 5555,           # 수신 포트
    "watchdog": 1.0,        # 워치독 타임아웃(초)
    "deg_max": 30.0,        # 송신 각도의 풀스케일(±deg_max)
    "center_norm": -0.2,    # 실제 직진이 되는 정규화 스티어 값
    "verbose": True         # 기본 로그 출력 여부
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
        description="UDP receiver -> Jetracer driver (with center bias). "
                    "옵션을 생략하면 CONFIG 기본값으로 동작합니다."
    )
    p.add_argument("--bind-ip", default=CONFIG["bind_ip"], help=f"수신 바인드 IP (기본: {CONFIG['bind_ip']})")
    p.add_argument("--port", type=int, default=CONFIG["port"], help=f"수신 포트 (기본: {CONFIG['port']})")
    p.add_argument("--watchdog", type=float, default=CONFIG["watchdog"], help=f"워치독 타임아웃 초 (기본: {CONFIG['watchdog']}s)")
    p.add_argument("--deg-max", type=float, default=CONFIG["deg_max"], help=f"송신 각도의 풀스케일 (기본: ±{CONFIG['deg_max']} deg)")
    p.add_argument("--center-norm", type=float, default=CONFIG["center_norm"],
                   help=f"실제 직진이 되는 정규화 스티어 값 (기본: {CONFIG['center_norm']:+.2f})")
    p.add_argument("--verbose", action="store_true" if not CONFIG["verbose"] else "store_false",
                   help="수신 로그 출력 토글 (기본: CONFIG 설정값)")
    # 설명: verbose 기본 True면 --verbose 를 넣으면 False로 토글, 기본 False면 넣으면 True로 토글
    return p

def main():
    parser = build_parser()
    args = parser.parse_args()

    # verbose 기본 처리: 위 토글 로직으로 인해 값이 비어 있을 수 있으니 보정
    if hasattr(args, "verbose"):
        # store_true/store_false 조합으로 인한 None 방지
        pass
    else:
        args.verbose = CONFIG["verbose"]

    car = NvidiaRacecar()

    # === 시작 시: 실제 센터로 정렬, 스로틀 0 ===
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
            # 수신
            try:
                data, addr = sock.recvfrom(64)
            except socket.timeout:
                data = None

            if data and len(data) >= PACKET_SIZE:
                throttle, steering_deg, seq = struct.unpack(FMT, data[:PACKET_SIZE])

                # NaN/Inf 방어
                if not (is_finite(throttle) and is_finite(steering_deg)):
                    continue

                # 시퀀스 관리(원하면 중복/역행 필터 추가 가능)
                last_seq = seq if last_seq is None else seq
                last_rx_ts = time.time()

                # === 정규화 + 센터 바이어스 적용 ===
                raw_norm = steering_deg / float(args.deg_max)
                steer_norm = clamp(raw_norm + args.center_norm, -1.0, 1.0)
                thr = clamp(throttle, 0.0, 1.0)

                # 적용 (NvidiaRacecar 내부에서 gain/soft-start/offset 처리 주의)
                car.steering = steer_norm
                car.throttle = thr

                if args.verbose:
                    print(f"seq={seq} thr={thr:+.3f} deg={steering_deg:+.1f} "
                          f"raw_norm={raw_norm:+.3f} -> steer_norm={steer_norm:+.3f}")

            # === 워치독: 미수신 시 실제 센터로 정렬 + 정지 ===
            now = time.time()
            if (now - last_rx_ts) > args.watchdog:
                if (car.steering != args.center_norm) or (car.throttle != 0.0):
                    car.steering = args.center_norm
                    car.throttle = 0.0
                    if args.verbose:
                        print(f"[watchdog] no packets > {args.watchdog:.2f}s → steering={args.center_norm:+.3f}, throttle=0.0")
                last_rx_ts = now  # 반복 로그 억제

    except KeyboardInterrupt:
        pass
    finally:
        try:
            car.throttle = 0.0
            car.steering = args.center_norm   # 종료 시도 실제 센터로
        except Exception:
            pass
        sock.close()
        print("exiting... motors stopped (centered).")

if __name__ == "__main__":
    main()
