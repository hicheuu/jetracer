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


def run_udp(log_queue, stop_event, auto_calibrate=False, target_velocity=5.0):
    """
    UDP 패킷을 수신하여 추상화된 제어 명령(스티어링, 속도)으로 변환 후 MUX로 전달합니다.
    """
    # 전송용 UDS 소켓 생성
    udsock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    # 수신용 UDP 소켓 생성 및 바인딩
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    log_queue.put({"type": "LOG", "src": "UDP", "msg": f"Listening on {UDP_IP}:{UDP_PORT}"})
    if auto_calibrate:
        log_queue.put({"type": "LOG", "src": "UDP", "msg": f"Auto-calibration ENABLED (Target: {target_velocity} m/s)"})

    last_rx_time = time.time()
    last_log_time = 0.0
    last_calib_time = 0.0
    DEADZONE = 0.05 # m/s

    try:
        while not stop_event.is_set():
            try:
                # 비블로킹 방식으로 UDP 패킷 수신
                data, _ = sock.recvfrom(64)

                # ROS 송신부 포맷 대응: !fifi (16 bytes) 또는 !fffI (16 bytes)
                if len(data) >= 16:
                    try:
                        raw_steer, raw_speed, obs_speed, seq = struct.unpack("!fffI", data[:16])
                    except:
                        raw_steer, raw_speed, obs_speed, seq = struct.unpack("!fifi", data[:16])

                    if not math.isfinite(raw_speed):
                        continue

                    steer_cmd = clamp(raw_steer * STEER_GAIN, -1.0, 1.0)
                    speed_cmd = float(raw_speed)

                    # 4. 실시간 자동 보정 수행
                    # - 자동 보정이 켜져 있고
                    # - 차량 명령 속도가 5.0 근처(최고속도)이며
                    # - 실제 속도 데이터가 유효할 때 작동
                    now = time.time()
                    if auto_calibrate and speed_cmd >= 4.5 and obs_speed > 0:
                        if now - last_calib_time > 0.5: # 0.5초마다 보정 판단
                            error = target_velocity - obs_speed
                            if abs(error) > DEADZONE:
                                event = "speed5_up" if error > 0 else "speed5_down"
                                udsock.sendto(
                                    json.dumps({"src": "auto", "event": event}).encode(),
                                    SOCK_PATH
                                )
                                last_calib_time = now

                    # 5. MUX에 제어 메시지 송신 (obs_speed 포함)
                    udsock.sendto(
                        json.dumps({
                            "src": "udp",
                            "steer": steer_cmd,
                            "speed": speed_cmd,
                            "obs_speed": obs_speed
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
