import socket
import struct
import time
import json
import math
import multiprocessing

# =========================
# UDS 설정
# =========================
SOCK_PATH = "/tmp/jetracer_ctrl.sock"

# =========================
# UDP 설정
# =========================
UDP_IP = "0.0.0.0"
UDP_PORT = 5555

# =========================
# 제어 파라미터
# =========================
STEER_GAIN = 2.0
WATCHDOG_TIMEOUT = 1.0

# speed 유효 범위 (⭐ 핵심 안전장치)
SPEED_MIN = 0.0
SPEED_MAX = 5.0


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def run_udp(log_queue, stop_event):
    """
    UDP receiver that sends abstract control commands to MUX.
    
    UDS message format:
    {
        "src": "udp",
        "steer": float,  // -1.0 to 1.0
        "speed": float   // 0.0 to 5.0
    }
    """
    # UDS 소켓은 함수 내에서 생성 (multiprocessing 호환)
    udsock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    log_queue.put({"type": "LOG", "src": "UDP", "msg": f"Listening on {UDP_IP}:{UDP_PORT}"})
    log_queue.put({"type": "LOG", "src": "UDP", "msg": f"format=!ffI | speed range=[{SPEED_MIN}, {SPEED_MAX}]"})

    last_rx_time = time.time()
    last_log_time = 0.0

    # 마지막 유효 명령 상태
    last_steer_cmd = 0.0
    last_speed_cmd = 0.0

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

                    # ===== speed (pass through) =====
                    speed_cmd = raw_speed

                    # 상태 갱신
                    last_steer_cmd = steer_cmd
                    last_speed_cmd = speed_cmd

                    # ===== UDS 송신 =====
                    udsock.sendto(
                        json.dumps({
                            "src": "udp",
                            "steer": steer_cmd,
                            "speed": speed_cmd
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
                             "msg": f"seq={seq:<5} speed={speed_cmd:.2f} steer={steer_cmd:+.3f}"
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
    class PrintQueue:
        def put(self, item):
            if item.get("type") == "LOG":
                print(f"[{item['src']}] {item['msg']}")

    stop = multiprocessing.Event()
    try:
        run_udp(PrintQueue(), stop)
    except KeyboardInterrupt:
        stop.set()
