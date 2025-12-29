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


def run_udp(log_queue, stop_event, auto_calibrate=False, target_velocity=5.0, shared_inc=None, shared_dec=None, **kwargs):
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
    log_queue.put({"type": "LOG", "src": "UDP", "msg": f"DEBUG: auto_calibrate={auto_calibrate}, target={target_velocity}, threshold={kwargs.get('threshold')}"})
    if auto_calibrate:
        log_queue.put({"type": "LOG", "src": "UDP", "msg": "Auto-calibration ENABLED"})

    last_rx_time = time.time()
    last_log_time = 0.0
    # 자동 보정용 윈도우 데이터 저장 (collections.deque 활용)
    from collections import deque
    # runner.py에서 'window_packets'로 전달받음 (기본 33개 = 약 1초)
    window_len = kwargs.get("window_packets", 33)
    speed_window = deque(maxlen=window_len)
    
    # 초기값은 kwargs에서 가져오되, 루프 내에서는 shared_inc/dec를 참조함
    initial_inc = kwargs.get("increment", 0.001)   # Stall Recovery용
    initial_dec = kwargs.get("decrement", -0.001)  # Speed Limit용
    threshold_v = kwargs.get("threshold", 3.2)
    
    last_seq = None
    packet_counter = 0     # 1Hz 보정 주기를 맞추기 위한 카운터
    last_diag_time = 0.0   # 자동보정 진단용 타이머 추가
    inc_count = 0          # 증가(정지 복구) 횟수
    dec_count = 0          # 감소(과속 방지) 횟수

    try:
        while not stop_event.is_set():
            now = time.time()
            
            # 1초 주기 하트비트 진단 로그 (데이터 수신 여부와 상관없이 출력)
            if now - last_diag_time > 1.0:
                avg_1s = sum([s for t, s in speed_window]) / len(speed_window) if speed_window else 0.0
                log_queue.put({
                    "type": "LOG", 
                    "src": "UDP", 
                    "msg": f"[DIAG] Alive. Calib={auto_calibrate} Win={len(speed_window)} Avg1s={avg_1s:.2f} last_rx={now-last_rx_time:.1f}s"
                })
                last_diag_time = now

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

                    # 패킷 누락 감지 (Sequence Number 추적)
                    lost_packets = 0
                    if last_seq is not None:
                        expected = (last_seq + 1) & 0xFFFFFFFF # 32bit wrap-around 고려
                        if seq != expected:
                            lost_packets = (seq - expected) & 0xFFFFFFFF
                            if lost_packets < 1000: # 비정상적으로 큰 값은 무시 (재시작 등)
                                log_queue.put({"type": "LOG", "src": "UDP", "msg": f"Packet loss detected: {lost_packets} packets (seq {last_seq}->{seq})"})
                            else:
                                lost_packets = 0 # 예외 케이스 초기화
                    last_seq = seq

                    if not math.isfinite(raw_speed):
                        continue

                    steer_cmd = clamp(raw_steer * STEER_GAIN, -1.0, 1.0)
                    speed_cmd = float(raw_speed)

                    now = time.time()

                    # 4. 실시간 자동 보정 수행 (패킷 카운트 방식)
                    if auto_calibrate:
                        # 윈도우에 현재 데이터 추가 (deque maxlen에 의해 자동 관리됨)
                        speed_window.append(obs_speed)
                        packet_counter += 1
                        
                        # 차량에 명령 속도가 들어오고 있을 때만 판단 (0.5 m/s 이상)
                        if speed_cmd >= 0.5:
                            # 33개 패킷(약 1초)이 쌓였다면 보정 로직 실행
                            if packet_counter >= window_len:
                                avg_speed = sum(speed_window) / len(speed_window)
                                
                                adjust_msg = ""
                                final_delta = 0.0
                                reason = ""

                                # 1. 과속 보정 (임계값 3.2 초과 시 감속)
                                if avg_speed > threshold_v:
                                    final_delta = shared_dec.value if shared_dec else initial_dec
                                    adjust_msg = f"Speed Limit: Avg({avg_speed:.2f}) > {threshold_v}"
                                    reason = "speed_limit"
                                    dec_count += 1
                                
                                # 2. 저속/정지 보정 (명령은 4.5 이상인데 실제 평균이 0.5 이하일 때 가속)
                                elif speed_cmd >= 4.5 and avg_speed <= 0.5:
                                    final_delta = shared_inc.value if shared_inc else initial_inc
                                    adjust_msg = f"Stall Recovery: Avg({avg_speed:.2f}) <= 0.5"
                                    reason = "stall_recovery"
                                    inc_count += 1

                                if final_delta != 0.0:
                                    udsock.sendto(
                                        json.dumps({
                                            "src": "auto", 
                                            "event": "speed5_adjust", 
                                            "delta": final_delta,
                                            "reason": reason,
                                            "threshold": threshold_v,
                                            "inc": shared_inc.value if shared_inc else initial_inc,
                                            "dec": shared_dec.value if shared_dec else initial_dec
                                        }).encode(),
                                        SOCK_PATH
                                    )
                                    log_queue.put({
                                        "type": "LOG", 
                                        "src": "UDP", 
                                        "msg": f"Auto-Calib: {adjust_msg} -> Adjust {final_delta:+.5f}"
                                    })
                                
                                packet_counter = 0 # 카운터 초기화

                    # 5. MUX에 제어 메시지 송신 (분석을 위해 cmd_speed, threshold, lost_packets 포함)
                    udsock.sendto(
                        json.dumps({
                            "src": "udp",
                            "steer": steer_cmd,
                            "speed": speed_cmd,
                            "obs_speed": obs_speed,
                            "threshold": threshold_v,
                            "lost_packets": lost_packets,
                            "inc": shared_inc.value if shared_inc else initial_inc,
                            "dec": shared_dec.value if shared_dec else initial_dec
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
            except Exception as e:
                log_queue.put({"type": "LOG", "src": "UDP", "msg": f"CRITICAL LOOP ERROR: {e}"})
                time.sleep(0.1)

            # Watchdog: 일정 시간 동안 수신이 없으면 경고
            if time.time() - last_rx_time > WATCHDOG_TIMEOUT:
                last_rx_time = time.time()
                log_queue.put({"type": "LOG", "src": "UDP", "msg": "watchdog timer triggered"})

            time.sleep(0.005)

    except Exception as e:
        log_queue.put({"type": "LOG", "src": "UDP", "msg": f"UDP PROCESS CRASHED: {e}"})
    finally:
        total = inc_count + dec_count
        log_queue.put({"type": "LOG", "src": "UDP", "msg": f"Auto-Calib 종료: 총 {total}회 보정 (증가: {inc_count}회, 감소: {dec_count}회)"})
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
