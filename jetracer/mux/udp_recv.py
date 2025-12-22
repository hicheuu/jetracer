#!/usr/bin/env python3
"""
Simple UDP Receiver -> UDS Sender
- Format: !fiI (float steer, int speed, uint seq)
- UDP watchdog에서는 제어값을 변경하지 않음
- 최종 정지는 mux.py watchdog에 위임
"""

import socket
import struct
import time
import json

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
STEER_GAIN = 1.0

# =========================
# Throttle 매핑 설정
# =========================

SPEED_1_THROTTLE = 0.225  # speed=1일 때 throttle 값
SPEED_5_THROTTLE = 0.235  # speed=5일 때 throttle 값


MAX_THROTTLE = 0.36
ESC_NEUTRAL = 0.12

WATCHDOG_TIMEOUT = 1.0


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def speed_to_throttle(speed):
    if speed == 0:
        return ESC_NEUTRAL

    slope = (SPEED_5_THROTTLE - SPEED_1_THROTTLE) / (5.0 - 1.0)
    intercept = SPEED_1_THROTTLE - slope * 1.0

    throttle = slope * float(speed) + intercept
    return clamp(throttle, ESC_NEUTRAL, ESC_NEUTRAL + MAX_THROTTLE)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    print(f"[UDP] Listening on {UDP_IP}:{UDP_PORT}")
    print(
        f"[UDP] speed=1 -> thr={SPEED_1_THROTTLE:.3f}, "
        f"speed=5 -> thr={SPEED_5_THROTTLE:.3f}"
    )

    last_rx_time = time.time()

    # ⭐ 마지막 유효 명령 상태 유지
    last_steer_cmd = 0.0
    last_throttle_cmd = ESC_NEUTRAL

    try:
        while True:
            try:
                data, _ = sock.recvfrom(64)

                if len(data) >= 12:
                    raw_steer, raw_speed, seq = struct.unpack("!fiI", data[:12])

                    # ===== steering =====
                    steer_val = raw_steer * STEER_GAIN
                    steer_cmd = clamp(steer_val, -1.0, 1.0)

                    # ===== throttle =====
                    throttle_cmd = speed_to_throttle(raw_speed)

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
                            f"speed={raw_speed} -> thr={throttle_cmd:.3f} "
                            f"steer={steer_cmd:+.3f}"
                        )

            except BlockingIOError:
                pass
            except struct.error:
                pass

            # ===== watchdog (⭐ 제어값 변경 금지) =====
            if time.time() - last_rx_time > WATCHDOG_TIMEOUT:
                # 여기서는 아무 제어 명령도 보내지 않음
                # → mux.py watchdog이 최종 정지 판단
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
