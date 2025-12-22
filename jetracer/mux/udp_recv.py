#!/usr/bin/env python3
"""
Simple UDP Receiver -> UDS Sender
- Format: !ffI (float steer, float speed, uint seq)
- UDP watchdog에서는 제어값을 변경하지 않음
- 최종 정지는 mux.py watchdog에 위임
- speed sanity check 포함 (안전)
"""

import socket
import struct
import time
import json
import math
import argparse

# =========================
# CLI 인자
# =========================
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--speed5-throttle",
        type=float,
        required=True,
        help="Throttle value corresponding to speed=5.0"
    )
    return parser.parse_args()


# =========================
# UDS 설정
# =========================
SOCK_PATH = "/tmp/jetracer_ctrl.sock"
udsock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

# =========================
# UDP 설정
# =========================
UDP_IP = "0.0.0.0"
UDP_PORT = 5555

# =========================
# 제어 파라미터
# =========================
STEER_GAIN = 2.0

MAX_THROTTLE = 0.36
ESC_NEUTRAL = 0.12

WATCHDOG_TIMEOUT = 1.0

# speed 유효 범위 (⭐ 핵심 안전장치)
SPEED_MIN = 0.0
SPEED_MAX = 5.0


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def speed_to_throttle(speed: float,
                      speed1_thr: float,
                      speed5_thr: float) -> float:
    if speed <= 0.0:
        return ESC_NEUTRAL

    slope = (speed5_thr - speed1_thr) / (5.0 - 1.0)
    intercept = speed1_thr - slope * 1.0

    throttle = slope * speed + intercept
    return clamp(throttle, ESC_NEUTRAL, ESC_NEUTRAL + MAX_THROTTLE)


def main():
    args = parse_args()

    SPEED_5_THROTTLE = args.speed5_throttle
    SPEED_1_THROTTLE = SPEED_5_THROTTLE - 0.01  # ⭐ 자동 계산

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    print(f"[UDP] Listening on {UDP_IP}:{UDP_PORT}")
    print(
        f"[UDP] format=!ffI | "
        f"speed=1 -> thr={SPEED_1_THROTTLE:.3f}, "
        f"speed=5 -> thr={SPEED_5_THROTTLE:.3f}"
    )

    last_rx_time = time.time()

    # 마지막 유효 명령 상태
    last_steer_cmd = 0.0
    last_throttle_cmd = ESC_NEUTRAL

    try:
        while True:
            try:
                data, _ = sock.recvfrom(64)

                if len(data) >= 12:
                    raw_steer, raw_speed, seq = struct.unpack("!ffI", data[:12])

                    # ===== speed sanity check =====
                    if not math.isfinite(raw_speed):
                        print("[UDP] invalid speed (nan/inf), drop")
                        continue

                    if raw_speed < SPEED_MIN or raw_speed > SPEED_MAX:
                        print(f"[UDP] invalid speed={raw_speed:.3f}, drop")
                        continue

                    # ===== steering =====
                    steer_val = raw_steer * STEER_GAIN
                    steer_cmd = clamp(steer_val, -1.0, 1.0)

                    # ===== throttle =====
                    throttle_cmd = speed_to_throttle(
                        raw_speed,
                        SPEED_1_THROTTLE,
                        SPEED_5_THROTTLE
                    )

                    # 상태 갱신
                    last_steer_cmd = steer_cmd
                    last_throttle_cmd = throttle_cmd

                    # ===== UDS 송신 =====
                    udsock.sendto(
                        json.dumps({
                            "src": "udp",
                            "steer": steer_cmd,
                            "throttle": throttle_cmd
                        }).encode(),
                        SOCK_PATH
                    )

                    last_rx_time = time.time()

                    if seq % 20 == 0:
                        print(
                            f"[UDP→MUX] seq={seq:<5} "
                            f"speed={raw_speed:.2f} -> thr={throttle_cmd:.3f} "
                            f"steer={steer_cmd:+.3f}"
                        )

            except BlockingIOError:
                pass
            except struct.error as e:
                print(f"[UDP] struct error: {e}")

            # ===== watchdog (제어 개입 금지) =====
            if time.time() - last_rx_time > WATCHDOG_TIMEOUT:
                last_rx_time = time.time()
                print("[UDP] watchdog (no control override)")

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[UDP] stopping")
    finally:
        sock.close()
        udsock.close()


if __name__ == "__main__":
    main()
