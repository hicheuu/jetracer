import socket
import struct
import time
import json
import math
import argparse
import multiprocessing
import os
import sys

from jetracer.core.nvidia_racecar import load_config

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

WATCHDOG_TIMEOUT = 1.0

# speed 유효 범위 (⭐ 핵심 안전장치)
SPEED_MIN = 0.0
SPEED_MAX = 5.0


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)




def speed_to_throttle(speed: float,
                      speed1_thr: float,
                      speed5_thr: float,
                      neutral: float) -> float:
    if speed <= 0.0:
        return neutral

    slope = (speed5_thr - speed1_thr) / (5.0 - 1.0)
    intercept = speed1_thr - slope * 1.0

    throttle = slope * speed + intercept
    return clamp(throttle, neutral, neutral + MAX_THROTTLE)


def run_udp(log_queue, stop_event, speed5_throttle):
    config = load_config()
    ESC_NEUTRAL = config.get("throttle", {}).get("neutral", 0.12) if config else 0.12
    log_queue.put({"type": "LOG", "src": "UDP", "msg": f"Loaded ESC_NEUTRAL={ESC_NEUTRAL} from config"})
    
    SPEED_5_THROTTLE = speed5_throttle
    SPEED_1_THROTTLE = SPEED_5_THROTTLE - 0.01  # ⭐ 자동 계산

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    log_queue.put({"type": "LOG", "src": "UDP", "msg": f"Listening on {UDP_IP}:{UDP_PORT}"})
    log_queue.put({"type": "LOG", "src": "UDP", "msg": f"format=!ffI | speed=1 -> thr={SPEED_1_THROTTLE:.3f}, speed=5 -> thr={SPEED_5_THROTTLE:.3f}"})

    last_rx_time = time.time()
    last_log_time = 0.0

    # 마지막 유효 명령 상태
    last_steer_cmd = 0.0
    last_throttle_cmd = ESC_NEUTRAL

    try:
        while not stop_event.is_set():
            try:
                data, _ = sock.recvfrom(64)

                if len(data) >= 12:
                    raw_steer, raw_speed, seq = struct.unpack("!ffI", data[:12])

                    # ===== speed sanity check =====
                    if not math.isfinite(raw_speed):
                        log_queue.put({"type": "LOG", "src": "UDP", "msg": "invalid speed (nan/inf), drop"})
                        continue

                    if raw_speed < SPEED_MIN or raw_speed > SPEED_MAX:
                        log_queue.put({"type": "LOG", "src": "UDP", "msg": f"invalid speed={raw_speed:.3f}, drop"})
                        continue

                    # ===== steering =====
                    steer_val = raw_steer * STEER_GAIN
                    steer_cmd = clamp(steer_val, -1.0, 1.0)

                    # ===== throttle =====
                    throttle_cmd = speed_to_throttle(
                        raw_speed,
                        SPEED_1_THROTTLE,
                        SPEED_5_THROTTLE,
                        ESC_NEUTRAL
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
                    now = time.time()

                    # Throttled Logging (0.5s)
                    if now - last_log_time > 0.5:
                         log_queue.put({
                             "type": "LOG",
                             "src": "UDP",
                             "msg": f"seq={seq:<5} speed={raw_speed:.2f} -> thr={throttle_cmd:.3f} steer={steer_cmd:+.3f}"
                         })
                         last_log_time = now

            except BlockingIOError:
                pass
            except struct.error as e:
                 log_queue.put({"type": "LOG", "src": "UDP", "msg": f"struct error: {e}"})

            # ===== watchdog (제어 개입 금지) =====
            if time.time() - last_rx_time > WATCHDOG_TIMEOUT:
                last_rx_time = time.time()
                log_queue.put({"type": "LOG", "src": "UDP", "msg": "watchdog (no control override)"})

            time.sleep(0.005)

    except KeyboardInterrupt:
        pass
    finally:
        log_queue.put({"type": "LOG", "src": "UDP", "msg": "stopping"})
        sock.close()
        udsock.close()


if __name__ == "__main__":
    args = parse_args()
    
    class PrintQueue:
        def put(self, item):
            if item.get("type") == "LOG":
                print(f"[{item['src']}] {item['msg']}")

    stop = multiprocessing.Event()
    try:
        run_udp(PrintQueue(), stop, args.speed5_throttle)
    except KeyboardInterrupt:
        stop.set()
