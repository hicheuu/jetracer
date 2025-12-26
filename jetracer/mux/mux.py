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
    속도(0.0 ~ 5.0)를 NvidiaRacecar가 사용하는 정규화된 스로틀 값(0.0 ~ 1.0)으로 변환합니다.
    
    NvidiaRacecar의 공식(전진): 
    phys = neutral + (norm * (1.0 - neutral) * gain)
    
    역산 공식:
    norm = (phys - neutral) / ((1.0 - neutral) * gain)
    """
    if speed <= 0.0:
        return 0.0
    
    slope = (speed5_phys - speed1_phys) / (5.0 - 1.0)
    intercept = speed1_phys - slope * 1.0
    target_phys = slope * speed + intercept
    
    denominator = (1.0 - neutral) * gain
    if abs(denominator) < 1e-5:
        return 0.0
        
    normalized = (target_phys - neutral) / denominator
    
    return max(0.0, min(1.0, normalized))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--speed5-throttle",
        type=float,
        required=True,
        help="속도 5.0일 때의 물리적 스로틀 목표값 (예: 0.35)"
    )
    return parser.parse_args()


def run_mux(log_queue, stop_event, speed5_throttle):
    # 기존 소켓 제거
    if os.path.exists(SOCK_PATH):
        try:
            os.unlink(SOCK_PATH)
        except OSError:
            pass

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(SOCK_PATH)
    sock.setblocking(False)

    car = NvidiaRacecar()
    
    # 하드웨어 파라미터 추출
    ESC_NEUTRAL = car._throttle_neutral
    THR_GAIN = car.throttle_gain
    
    SPEED_5_PHYS = speed5_throttle
    SPEED_1_PHYS = SPEED_5_PHYS - 0.01  
    
    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"Mapped Speed Mapping: Neutral={ESC_NEUTRAL:.3f}, Gain={THR_GAIN:.2f}"})
    
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
            try:
                data, _ = sock.recvfrom(512)
                msg = json.loads(data.decode())
                src = msg.get("src")

                if msg.get("event") == "toggle":
                    mode = "udp" if mode == "joystick" else "joystick"
                    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"MODE → {mode}"})
                    log_queue.put({"type": "MODE", "mode": mode})
                    continue

                if msg.get("event") == "estop":
                    estop = not estop
                    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"ESTOP {'ON' if estop else 'OFF'}"})
                    continue

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

            if estop:
                car.steering = 0.0
                car.throttle = 0.0
                time.sleep(0.01)
                continue

            cmd = None
            src_used = None

            if mode == "udp" and last_udp and now - last_udp["ts"] < UDP_TIMEOUT:
                cmd = last_udp
                src_used = "UDP"
            elif mode == "joystick" and last_joy and now - last_joy["ts"] < JOY_TIMEOUT:
                cmd = last_joy
                src_used = "JOY"

            if cmd:
                car.steering = cmd["steer"]
                
                if "speed" in cmd:
                    # 새로운 NvidiaRacecar 공식에 맞춘 정교한 역산 적용
                    car.throttle = speed_to_normalized_throttle(
                        cmd["speed"],
                        SPEED_1_PHYS,
                        SPEED_5_PHYS,
                        ESC_NEUTRAL,
                        THR_GAIN
                    )
                else:
                    car.throttle = cmd["throttle"]
                
                if now - last_log_time > 0.5:
                    log_queue.put({
                        "type": "LOG", 
                        "src": "MUX", 
                        "msg": f"[{src_used}] steer={car.steering:+.2f} thr={car.throttle:+.3f} (norm)"
                    })
                    last_log_time = now
            else:
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
