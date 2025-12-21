#!/usr/bin/env python3
"""
Simple UDP Receiver -> UDS Sender
- Format: !fiI (float steer, int speed, uint seq)
- speed=5 -> throttle=0.339 유지
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
# 제어 파라미터 (기존 그대로)
# =========================
STEER_GAIN = 1.0
STEER_OFFSET = 0.0

SPEED_GAIN = 0.0438
MAX_THROTTLE = 0.36
ESC_NEUTRAL = 0.12

WATCHDOG_TIMEOUT = 1.0


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    print(f"[UDP] Listening on {UDP_IP}:{UDP_PORT}")
    print("[UDP] Format = !fiI | speed=5 -> thr=0.339")

    last_rx_time = time.time()

    try:
        while True:
            try:
                data, _ = sock.recvfrom(64)

                if len(data) >= 12:
                    raw_steer, raw_speed, seq = struct.unpack("!fiI", data[:12])

                    # ===== steering =====
                    steer_val = (raw_steer * STEER_GAIN) + STEER_OFFSET
                    steer_cmd = clamp(steer_val, -1.0, 1.0)

                    # ===== throttle =====
                    throttle_val = float(raw_speed) * SPEED_GAIN
                    if throttle_val != 0:
                        final_throttle = ESC_NEUTRAL + throttle_val
                    else:
                        final_throttle = ESC_NEUTRAL

                    throttle_cmd = clamp(
                        final_throttle,
                        ESC_NEUTRAL,
                        ESC_NEUTRAL + MAX_THROTTLE
                    )

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

            # ===== watchdog =====
            if time.time() - last_rx_time > WATCHDOG_TIMEOUT:
                udsock.sendto(
                    json.dumps({
                        "src": "udp",
                        "steer": 0.0,
                        "throttle": ESC_NEUTRAL
                    }).encode(),
                    SOCK_PATH
                )
                last_rx_time = time.time()
                print("[UDP] watchdog → neutral")

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[UDP] stopping")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
