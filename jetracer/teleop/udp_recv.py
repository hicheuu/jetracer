#!/usr/bin/env python3
"""
UDP receiver -> JetRacer driver
자이카 udp_recv.py의 수신 방식을 참고하여 개선된 버전
"""
import argparse
import math
import socket
import struct
import time
from typing import Optional, Tuple

from jetracer.core import NvidiaRacecar

# ==============================
# 기본 실행 설정
# ==============================
CONFIG = {
    "bind_ip": "0.0.0.0",
    "port": 5555,
    "watchdog": 1.0,
    "verbose": True,
    # 스티어링 설정
    "angle_scale": 50.0,        # INT 모드: steer_rad = angle_i / angle_scale
    "angle_invert": False,      # 스티어링 반전
    "steer_scale": 1.0,         # 수신값 → 정규화 스케일 (수신값이 이미 -1~1이면 1.0)
    # 스로틀 설정
    "speed_scale": 0.65,        # speed_i → throttle 변환 (speed_i * scale)
    "speed_max": 0.2,           # 최대 스로틀 값
}
# ==============================

# 패킷 포맷 (udp_recv.py와 동일)
FMT_INT = "!iiI"      # int32 angle_i, int32 speed_i, uint32 seq
FMT_FLOAT = "!fiI"    # float angle_rad, int32 speed_i, uint32 seq
PKT_SIZE_INT = struct.calcsize(FMT_INT)
PKT_SIZE_FLOAT = struct.calcsize(FMT_FLOAT)


def clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return lo if x < lo else hi if x > hi else x


def is_finite(x) -> bool:
    return (x is not None) and (not math.isnan(x)) and (not math.isinf(x))


def decode_packet(data: bytes, fmt_mode: str, angle_scale: float) -> Optional[Tuple[str, float, int, int]]:
    """
    패킷 디코딩
    Returns: (fmt, steer_rad, speed_i, seq) or None
    """
    if fmt_mode == "auto":
        # FLOAT 포맷 우선 시도
        if len(data) >= PKT_SIZE_FLOAT:
            try:
                angle_f, speed_i, seq = struct.unpack(FMT_FLOAT, data[:PKT_SIZE_FLOAT])
                if is_finite(angle_f):
                    return ("float", float(angle_f), int(speed_i), int(seq))
            except struct.error:
                pass
        # INT 포맷 시도
        if len(data) >= PKT_SIZE_INT:
            try:
                angle_i, speed_i, seq = struct.unpack(FMT_INT, data[:PKT_SIZE_INT])
                steer_rad = float(angle_i) / max(1e-6, angle_scale)
                return ("int", float(steer_rad), int(speed_i), int(seq))
            except struct.error:
                pass
        return None

    elif fmt_mode == "float":
        if len(data) < PKT_SIZE_FLOAT:
            return None
        angle_f, speed_i, seq = struct.unpack(FMT_FLOAT, data[:PKT_SIZE_FLOAT])
        return ("float", float(angle_f), int(speed_i), int(seq))

    else:  # int
        if len(data) < PKT_SIZE_INT:
            return None
        angle_i, speed_i, seq = struct.unpack(FMT_INT, data[:PKT_SIZE_INT])
        steer_rad = float(angle_i) / max(1e-6, angle_scale)
        return ("int", float(steer_rad), int(speed_i), int(seq))


def build_parser():
    p = argparse.ArgumentParser(
        description="UDP receiver -> JetRacer driver (자이카 호환 포맷)"
    )
    # 네트워크
    p.add_argument("--bind-ip", default=CONFIG["bind_ip"], 
                   help=f"수신 바인드 IP (기본: {CONFIG['bind_ip']})")
    p.add_argument("--port", type=int, default=CONFIG["port"], 
                   help=f"수신 포트 (기본: {CONFIG['port']})")
    
    # 패킷 포맷
    p.add_argument("--fmt", choices=["auto", "int", "float"], default="auto",
                   help="패킷 포맷 (기본: auto)")
    
    # 워치독
    p.add_argument("--watchdog", type=float, default=CONFIG["watchdog"],
                   help=f"워치독 타임아웃 초 (기본: {CONFIG['watchdog']}s)")
    
    # 스티어링 설정
    p.add_argument("--angle-scale", type=float, default=CONFIG["angle_scale"],
                   help=f"INT 모드: steer_rad = angle_i / angle_scale (기본: {CONFIG['angle_scale']})")
    p.add_argument("--angle-invert", action="store_true", default=CONFIG["angle_invert"],
                   help="스티어링 방향 반전")
    p.add_argument("--steer-scale", type=float, default=CONFIG["steer_scale"],
                   help=f"수신값 → 정규화 스케일 (수신값이 -1~1이면 1.0) (기본: {CONFIG['steer_scale']})")
    
    # 스로틀 설정
    p.add_argument("--speed-scale", type=float, default=CONFIG["speed_scale"],
                   help=f"speed_i → throttle 변환 (기본: {CONFIG['speed_scale']})")
    p.add_argument("--speed-max", type=float, default=CONFIG["speed_max"],
                   help=f"최대 스로틀 값 (기본: {CONFIG['speed_max']})")
    
    # 출력
    p.add_argument("--verbose", action="store_true", default=CONFIG["verbose"],
                   help="수신 로그 출력")
    p.add_argument("--throttle-sec", type=float, default=0.2,
                   help="로그 출력 간격 (초)")
    
    return p


def main():
    parser = build_parser()
    args = parser.parse_args()

    # 차량 초기화
    car = NvidiaRacecar()
    car.steering = 0.0  # nvidia_racecar.py의 steering_offset이 적용됨
    car.throttle = 0.12  # ESC 중립점

    # 소켓 초기화 (non-blocking)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512 * 1024)
    except OSError:
        pass
    sock.bind((args.bind_ip, args.port))
    sock.setblocking(False)

    # 상태 변수
    last_seq: int = -1
    last_rx_ts: float = time.time()
    last_print_ts: float = 0.0
    latest_data: Optional[Tuple[str, float, int, int, tuple]] = None
    # latest_data = (fmt, steer_rad, speed_i, seq, addr)

    print(f"[UDP] listening on {args.bind_ip}:{args.port}")
    print(f"[UDP] fmt={args.fmt} | INT: {FMT_INT} ({PKT_SIZE_INT}B) | FLOAT: {FMT_FLOAT} ({PKT_SIZE_FLOAT}B)")
    print(f"[MAP] angle_scale={args.angle_scale} steer_scale={args.steer_scale}")
    print(f"[MAP] speed_scale={args.speed_scale} speed_max={args.speed_max}")
    if args.verbose:
        print(f"[init] steering -> 0.0 (offset applied in nvidia_racecar), throttle -> 0.12 (neutral)")

    try:
        while True:
            # === 패킷 수신 (non-blocking, 모든 대기 패킷 처리) ===
            while True:
                try:
                    data, addr = sock.recvfrom(64)
                except BlockingIOError:
                    break
                except Exception as e:
                    print(f"[UDP] recvfrom error: {e}")
                    break

                decoded = decode_packet(data, args.fmt, args.angle_scale)
                if decoded is None:
                    continue

                fmt, steer_rad, speed_i, seq = decoded

                # Sequence 체크 (오래된 패킷 무시, 리셋 감지)
                if last_seq != -1 and seq <= last_seq:
                    if last_seq > 100000 and seq < 100:
                        # 시퀀스 리셋 감지 (새 세션)
                        print("[UDP] Sequence reset detected (new session)")
                        last_seq = -1
                    else:
                        continue  # 오래된 패킷 무시

                last_seq = seq
                latest_data = (fmt, steer_rad, speed_i, seq, addr)
                last_rx_ts = time.time()

            # === 최신 데이터로 차량 제어 ===
            if latest_data is not None:
                fmt, steer_rad, speed_i, seq, addr = latest_data

                # 스티어링 변환: 수신값 → 정규화 (-1 ~ 1)
                # steer_rad가 이미 -1~1 범위라면 steer_scale=1.0
                steer_norm = steer_rad * args.steer_scale
                if args.angle_invert:
                    steer_norm = -steer_norm
                steer_norm = clamp(steer_norm, -1.0, 1.0)

                # 스로틀 변환: speed_i → throttle
                throttle = clamp(float(speed_i) * args.speed_scale, 0.0, args.speed_max)
                
                # ESC 중립점 적용 (joystick.py와 동일한 방식)
                ESC_NEUTRAL = 0.12
                if throttle > 0:
                    throttle_cmd = ESC_NEUTRAL + throttle * (1.0 - ESC_NEUTRAL)
                else:
                    throttle_cmd = ESC_NEUTRAL

                car.steering = steer_norm
                car.throttle = throttle_cmd

                # 로그 출력
                now = time.time()
                if args.verbose and (now - last_print_ts) >= args.throttle_sec:
                    print(
                        f"[{fmt.upper()}] seq={seq} steer_raw={steer_rad:+.4f} "
                        f"-> norm={steer_norm:+.3f} | "
                        f"speed_i={speed_i} -> thr={throttle_cmd:.3f}"
                    )
                    last_print_ts = now

            # === 워치독 체크 ===
            now = time.time()
            if (now - last_rx_ts) > args.watchdog:
                if latest_data is not None:
                    car.steering = 0.0  # nvidia_racecar.py의 steering_offset이 적용됨
                    car.throttle = 0.12  # ESC 중립점
                    if args.verbose:
                        print(
                            f"[watchdog] no packets > {args.watchdog:.2f}s → "
                            f"steering=0.0 (centered), throttle=0.12"
                        )
                    latest_data = None
                    last_seq = -1
                last_rx_ts = now

            # CPU 부하 방지
            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[UDP] interrupted")
    finally:
        try:
            car.throttle = 0.12  # ESC 중립점
            car.steering = 0.0  # nvidia_racecar.py의 steering_offset이 적용됨
        except Exception:
            pass
        sock.close()
        print("[UDP] exiting... motors stopped (centered).")


if __name__ == "__main__":
    main()
