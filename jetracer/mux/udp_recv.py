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
    """
    주어진 값을 최소값과 최대값 사이로 제한합니다.
    """
    return max(min(maxn, n), minn)


def run_udp(log_queue, stop_event):
    """
    UDP 패킷을 수신하여 추상화된 제어 명령(스티어링, 속도)으로 변환 후 MUX로 전달합니다.
    
    수신 데이터 형식: !ffI (float steer, float speed, uint sequence)
    MUX UDS 메시지 형식: {"src": "udp", "steer": float, "speed": float}
    """
    # 전송용 UDS 소켓 생성
    udsock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    # 수신용 UDP 소켓 생성 및 바인딩
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    log_queue.put({"type": "LOG", "src": "UDP", "msg": f"Listening on {UDP_IP}:{UDP_PORT}"})
    log_queue.put({"type": "LOG", "src": "UDP", "msg": f"format=!ffI | speed range=[{SPEED_MIN}, {SPEED_MAX}]"})

    last_rx_time = time.time()
    last_log_time = 0.0

    try:
        while not stop_event.is_set():
            try:
                # 비블로킹 방식으로 UDP 패킷 수신
                data, _ = sock.recvfrom(64)

                if len(data) >= 12:
                    raw_steer, raw_speed, seq = struct.unpack("!ffI", data[:12])

                    # 1. 속도 유효성 검사 (NaN/Inf 및 범위 확인)
                    if not math.isfinite(raw_speed):
                        log_queue.put({"type": "LOG", "src": "UDP", "msg": "invalid speed (nan/inf), drop"})
                        continue

                    if raw_speed < SPEED_MIN or raw_speed > SPEED_MAX:
                        log_queue.put({"type": "LOG", "src": "UDP", "msg": f"invalid speed={raw_speed:.3f}, drop"})
                        continue

                    # 2. 스티어링 게인 적용 및 클리핑
                    steer_val = raw_steer * STEER_GAIN
                    steer_cmd = clamp(steer_val, -1.0, 1.0)

                    # 3. 속도 값 그대로 전달 (MUX에서 물리적 매핑 수행)
                    speed_cmd = raw_speed

                    # 4. MUX에 제어 메시지 송신
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

                    # 0.5초 주기로 로그 출력
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

            # Watchdog: 일정 시간 동안 수신이 없으면 경고
            if time.time() - last_rx_time > WATCHDOG_TIMEOUT:
                last_rx_time = time.time()
                log_queue.put({"type": "LOG", "src": "UDP", "msg": "watchdog timer triggered"})

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
