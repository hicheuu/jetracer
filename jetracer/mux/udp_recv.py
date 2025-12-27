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


def run_udp(log_queue, stop_event, auto_calibrate=False, target_velocity=5.0, **kwargs):
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
    # 자동 보정용 윈도우 데이터 저장 (collections.deque 활용)
    from collections import deque
    speed_window = deque()
    # 보정 주기 및 윈도우 시간 (runner.py에서 전달받은 값 사용)
    # kwargs가 없으면 기본값 적용 (3.0s, 0.005, 4.5 m/s)
    window_s = kwargs.get("window_duration", 3.0)
    adjust_delta = kwargs.get("increment", -0.005)
    threshold_v = kwargs.get("threshold", 3.5)
    last_calib_time = 0.0
    adjust_count = 0  # 자동 보정 횟수 카운터

    try:
        while not stop_event.is_set():
            try:
                # 비블로킹 방식으로 UDP 패킷 수신
                data, _ = sock.recvfrom(64)

                # ROS 송신부 포맷 대응: !fifi (16 bytes) 또는 !fffI (16 bytes)
                if len(data) >= 16:
                    # 1. 일단 !fffI (float speed)로 시도
                    s1, s2, s3, s4 = struct.unpack("!fffI", data[:16])
                    
                    # 2. 값의 신뢰성 체크: speed_cmd(s2)가 비정상적으로 크거나 
                    # 정수가 float으로 잘못 해석되어 아주 작은 경우(0 제외) !fifi로 전환
                    if (abs(s2) > 100.0) or (0 < abs(s2) < 1e-10):
                        try:
                            s1, s2, s3, s4 = struct.unpack("!fifi", data[:16])
                        except:
                            pass # 실패 시 v1,v2,v3,v4 유지

                    raw_steer, raw_speed, obs_speed, seq = s1, s2, s3, s4

                    if not math.isfinite(raw_speed):
                        continue

                    steer_cmd = clamp(raw_steer * STEER_GAIN, -1.0, 1.0)
                    speed_cmd = float(raw_speed)

                    now = time.time()

                    # 4. 실시간 자동 보정 수행 (윈도우 방식)
                    if auto_calibrate and obs_speed > 0:
                        # 윈도우에 현재 데이터 추가
                        speed_window.append((now, obs_speed))
                        
                        # 윈도우 범위를 벗어난 오래된 데이터 제거
                        while speed_window and now - speed_window[0][0] > window_s:
                            speed_window.popleft()

                        # 차량이 최고속도 명령 상태일 때만 보정 로직 작동 여부 판단
                        if speed_cmd >= 4.5:
                            # 충분한 데이터가 쌓였고(윈도우의 80% 이상), 
                            # 마지막 보정으로부터 최소 윈도우 시간만큼 지났을 때 수행
                            if (now - speed_window[0][0] >= window_s * 0.8) and (now - last_calib_time > window_s):
                                avg_speed = sum([s for t, s in speed_window]) / len(speed_window)
                                
                                # 평균 속도가 임계값(3.5)을 초과하면 스로틀 하향 보정 (속도 제한)
                                if avg_speed > threshold_v:
                                    udsock.sendto(
                                        json.dumps({
                                            "src": "auto", 
                                            "event": "speed5_adjust", 
                                            "delta": adjust_delta
                                        }).encode(),
                                        SOCK_PATH
                                    )
                                    log_queue.put({
                                        "type": "LOG", 
                                        "src": "UDP", 
                                        "msg": f"Auto-Calib: Avg({avg_speed:.2f}) > {threshold_v} -> Adjust {adjust_delta:+.3f}"
                                    })
                                    last_calib_time = now
                                    adjust_count += 1

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
                            "msg": f"seq={seq:<5} cmd_speed={speed_cmd:.2f} obs_speed={obs_speed:.3f} steer={steer_cmd:+.3f}"
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
        log_queue.put({"type": "LOG", "src": "UDP", "msg": f"Auto-Calib 종료: 총 {adjust_count}회 보정 수행 (delta: {adjust_delta:+.3f})"})
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
