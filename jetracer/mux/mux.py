import socket
import json
import time
import os
import multiprocessing
import argparse

from jetracer.core import NvidiaRacecar


# =========================
# UDS 설정
# =========================
SOCK_PATH = "/tmp/jetracer_ctrl.sock"

# =========================
# 타임아웃 설정
# =========================
JOY_TIMEOUT = 0.5
UDP_TIMEOUT = 1.2


def speed_to_normalized_throttle(speed: float, 
                                 speed1_phys: float, 
                                 speed5_phys: float, 
                                 neutral: float, 
                                 gain: float) -> float:
    """
    속도(0.0 ~ 5.0)를 NvidiaRacecar가 사용하는 정규화된 스로틀 값(-1.0 ~ 1.0)으로 변환합니다.
    
    이 함수는 사용자가 정의한 속도 1.0과 5.0 지점의 물리적 스로틀 값을 기반으로 
    선형 보간을 수행하며, 최종적으로 정규화된 제어 신호를 계산합니다.
    """
    if speed <= 0.0:
        return 0.0
    
    # 1. 속도 1.0과 5.0 사이의 선형 기울기 및 절편 계산
    slope = (speed5_phys - speed1_phys) / (5.0 - 1.0)
    intercept = speed1_phys - slope * 1.0
    
    # 2. 현재 속도에 해당하는 물리적 스로틀 목표값 계산
    target_phys = slope * speed + intercept
    
    # 3. 물리적 목표값을 정규화된 입력값으로 역변환
    # NvidiaRacecar 공식: phys = neutral + (norm * gain)
    # 역산 공식: norm = (phys - neutral) / gain
    normalized = (target_phys - neutral) / gain
    
    return max(0.0, min(1.0, normalized))


def parse_args():
    """
    명령행 인자를 파싱합니다.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--speed5-throttle",
        type=float,
        required=True,
        help="속도 5.0일 때의 물리적 스로틀 목표값 (예: 0.35)"
    )
    return parser.parse_args()


def run_mux(log_queue, stop_event, speed5_throttle):
    """
    MUX 메인 루프입니다. 조이스틱과 UDP로부터 명령을 읽어 우선순위에 따라 차량을 제어합니다.
    """
    # 기존 소켓 파일 제거
    if os.path.exists(SOCK_PATH):
        try:
            os.unlink(SOCK_PATH)
        except OSError:
            pass

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(SOCK_PATH)
    sock.setblocking(False)

    # 차량 객체 생성 (설정 로드 포함)
    car = NvidiaRacecar()
    
    # 매핑 파라미터 준비
    ESC_NEUTRAL = car._throttle_neutral
    THR_GAIN = car.throttle_gain
    SPEED_5_PHYS = speed5_throttle
    SPEED_1_PHYS = SPEED_5_PHYS - 0.01  # 사용자의 기존 자동 계산 로직 유지
    
    log_queue.put({"type": "LOG", "src": "MUX", "msg": "Started with Piecewise Linear Speed Mapping"})
    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"Calibration: S1={SPEED_1_PHYS:.3f}, S5={SPEED_5_PHYS:.3f}, Neutral={ESC_NEUTRAL:.3f}"})
    
    car.steering = 0.0
    car.throttle = 0.0

    mode = "joystick"
    estop = False

    last_joy = None
    last_udp = None
    last_log_time = 0.0

    log_queue.put({"type": "MODE", "mode": mode})

    try:
        while not stop_event.is_set():
            # ===== 수신부 =====
            try:
                data, _ = sock.recvfrom(512)
                msg = json.loads(data.decode())
                src = msg.get("src")

                # 모드 전환 이벤트
                if msg.get("event") == "toggle":
                    mode = "udp" if mode == "joystick" else "joystick"
                    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"MODE → {mode}"})
                    log_queue.put({"type": "MODE", "mode": mode})
                    continue

                # 비상 정지 이벤트
                if msg.get("event") == "estop":
                    estop = not estop
                    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"ESTOP {'ON' if estop else 'OFF'}"})
                    continue

                # 일반 명령 저장
                msg["ts"] = time.time()
                if src == "joystick":
                    last_joy = msg
                elif src == "udp":
                    last_udp = msg

            except BlockingIOError:
                pass
            except Exception as e:
               log_queue.put({"type": "LOG", "src": "MUX", "msg": f"Error: {e}"})

            now = time.time()

            # 비상 정지 처리
            if estop:
                car.steering = 0.0
                car.throttle = 0.0
                time.sleep(0.01)
                continue

            cmd = None
            src_used = None

            # 명령 우선순위 및 타임아웃 결정
            if mode == "udp" and last_udp and now - last_udp["ts"] < UDP_TIMEOUT:
                cmd = last_udp
                src_used = "UDP"
            elif mode == "joystick" and last_joy and now - last_joy["ts"] < JOY_TIMEOUT:
                cmd = last_joy
                src_used = "JOY"

            # 하드웨어 명령 적용
            if cmd:
                car.steering = cmd["steer"]
                
                if "speed" in cmd:
                    # UDP speed -> Normalized Throttle 변환
                    car.throttle = speed_to_normalized_throttle(
                        cmd["speed"],
                        SPEED_1_PHYS,
                        SPEED_5_PHYS,
                        ESC_NEUTRAL,
                        THR_GAIN
                    )
                else:
                    # Joystick은 이미 정규화된 값(-1.0 ~ 1.0)을 보냄
                    car.throttle = cmd["throttle"]
                
                # 로깅 (0.5초 주기)
                if now - last_log_time > 0.5:
                    log_queue.put({
                        "type": "LOG", 
                        "src": "MUX", 
                        "msg": f"[{src_used}] steer={car.steering:+.2f} thr={car.throttle:+.3f} (norm)"
                    })
                    last_log_time = now
            else:
                # 명령 없을 시 정지
                car.steering = 0.0
                car.throttle = 0.0

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        car.steering = 0.0
        car.throttle = 0.0
        sock.close()
        if os.path.exists(SOCK_PATH):
            os.unlink(SOCK_PATH)
        log_queue.put({"type": "LOG", "src": "MUX", "msg": "stopped"})


if __name__ == "__main__":
    args = parse_args()
    
    class PrintQueue:
        def put(self, item):
            if item.get("type") == "LOG":
                print(f"[{item['src']}] {item['msg']}")
            elif item.get("type") == "MODE":
                 print(f"[{item['mode']}] MODE CHANGE")

    stop = multiprocessing.Event()
    try:
        run_mux(PrintQueue(), stop, args.speed5_throttle)
    except KeyboardInterrupt:
        stop.set()
