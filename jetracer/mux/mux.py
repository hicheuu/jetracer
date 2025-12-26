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

# =========================
# 속도 → 노멀라이즈 스로틀 변환
# =========================
def norm_speed_to_throttle(speed: float) -> float:
    """
    Convert absolute speed (0.0 ~ 5.0) to normalized throttle (0.0 ~ 1.0).
    Note: The actual physical mapping (gain, neutral) is handled by NvidiaRacecar.
    """
    if speed <= 0.0:
        return 0.0
    
    # 단순 선형 매핑 예시 (0.0 ~ 5.0 -> 0.0 ~ 1.0)
    # 필요에 따라 비선형 매핑이나 정밀 보간을 사용할 수 있음.
    normalized = speed / 5.0
    return max(0.0, min(1.0, normalized))


def parse_args():
    parser = argparse.ArgumentParser()
    # speed5-throttle은 이제 NvidiaRacecar의 gain으로 흡수되거나 
    # MUX에서 전체적인 스케일링 용도로 사용할 수 있음.
    # 여기서는 호환성을 위해 유지하거나 제거할 수 있음. 
    # 일단 '최대 스피드 가중치' 정도로 이해하고 유지.
    parser.add_argument(
        "--max-speed",
        type=float,
        default=5.0,
        help="Speed value that maps to 1.0 normalized throttle"
    )
    return parser.parse_args()


def run_mux(log_queue, stop_event, max_speed):
    # 소켓 초기화
    if os.path.exists(SOCK_PATH):
        try:
            os.unlink(SOCK_PATH)
        except OSError:
            pass

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(SOCK_PATH)
    sock.setblocking(False)

    # NvidiaRacecar는 이제 -1.0 ~ 1.0의 표준 입력을 받음
    car = NvidiaRacecar()
    
    log_queue.put({"type": "LOG", "src": "MUX", "msg": "Started with Normalized Control Architecture"})
    
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
            # ===== 수신 =====
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
                    # UDP speed -> Normalized Throttle (0.0 ~ 1.0)
                    car.throttle = cmd["speed"] / max_speed
                else:
                    # Joystick Normalized Throttle (-1.0 ~ 1.0)
                    car.throttle = cmd["throttle"]
                
                if now - last_log_time > 0.5:
                    log_queue.put({
                        "type": "LOG", 
                        "src": "MUX", 
                        "msg": f"{src_used} steer={car.steering:+.2f} thr={car.throttle:+.2f} (norm)"
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
        run_mux(PrintQueue(), stop, args.max_speed)
    except KeyboardInterrupt:
        stop.set()
