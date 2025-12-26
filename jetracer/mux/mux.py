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
UDP_TIMEOUT = 1.2   # ← UDP watchdog보다 살짝 큼

# =========================
# 속도 → 스로틀 변환 파라미터
# =========================
MAX_THROTTLE = 0.36


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def speed_to_throttle(speed: float,
                      speed1_thr: float,
                      speed5_thr: float,
                      neutral: float) -> float:
    """
    Convert abstract speed (0.0 ~ 5.0) to physical throttle value.
    """
    if speed <= 0.0:
        return neutral

    slope = (speed5_thr - speed1_thr) / (5.0 - 1.0)
    intercept = speed1_thr - slope * 1.0

    throttle = slope * speed + intercept
    return clamp(throttle, neutral, neutral + MAX_THROTTLE)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--speed5-throttle",
        type=float,
        required=True,
        help="Throttle value corresponding to speed=5.0"
    )
    return parser.parse_args()


def run_mux(log_queue, stop_event, speed5_throttle):
    # 소켓 초기화
    if os.path.exists(SOCK_PATH):
        try:
            os.unlink(SOCK_PATH)
        except OSError:
            pass

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(SOCK_PATH)
    sock.setblocking(False)

    car = NvidiaRacecar()
    ESC_NEUTRAL = car._throttle_neutral
    
    # speed → throttle 매핑 파라미터
    SPEED_5_THROTTLE = speed5_throttle
    SPEED_1_THROTTLE = SPEED_5_THROTTLE - 0.01  # ⭐ 자동 계산
    
    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"Loaded ESC_NEUTRAL={ESC_NEUTRAL} from config"})
    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"speed=1 -> thr={SPEED_1_THROTTLE:.3f}, speed=5 -> thr={SPEED_5_THROTTLE:.3f}"})
    
    car.steering = 0.0
    car.throttle = ESC_NEUTRAL

    mode = "joystick"   # joystick | udp
    estop = False

    last_joy = None
    last_udp = None
    
    last_log_time = 0.0

    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"started | mode={mode}"})
    # Initial mode update
    log_queue.put({"type": "MODE", "mode": mode})

    try:
        while not stop_event.is_set():
            # ===== 수신 =====
            try:
                data, _ = sock.recvfrom(512)
                msg = json.loads(data.decode())
                src = msg.get("src")

                # 이벤트
                if msg.get("event") == "toggle":
                    # MUX decides mode toggling
                    mode = "udp" if mode == "joystick" else "joystick"
                    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"MODE → {mode}"})
                    log_queue.put({"type": "MODE", "mode": mode})
                    continue

                if msg.get("event") == "estop":
                    estop = not estop
                    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"ESTOP {'ON' if estop else 'OFF'}"})
                    continue

                # 명령
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

            # ===== estop =====
            if estop:
                car.steering = 0.0
                car.throttle = ESC_NEUTRAL
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
                
                # ===== throttle 처리 =====
                if "speed" in cmd:
                    # UDP에서 온 speed → throttle 변환
                    car.throttle = speed_to_throttle(
                        cmd["speed"],
                        SPEED_1_THROTTLE,
                        SPEED_5_THROTTLE,
                        ESC_NEUTRAL
                    )
                else:
                    # Joystick에서 온 throttle 직접 사용
                    car.throttle = cmd["throttle"]
                
                # Log throttling (0.5s)
                if now - last_log_time > 0.5:
                    log_queue.put({
                        "type": "LOG", 
                        "src": "MUX", 
                        "msg": f"{src_used} steer={cmd['steer']:+.3f} thr={car.throttle:.3f}"
                    })
                    last_log_time = now
            else:
                car.steering = 0.0
                car.throttle = ESC_NEUTRAL

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    except Exception as e:
         log_queue.put({"type": "LOG", "src": "MUX", "msg": f"Exception: {e}"})
    finally:
        car.steering = 0.0
        car.throttle = ESC_NEUTRAL
        sock.close()
        if os.path.exists(SOCK_PATH):
            os.unlink(SOCK_PATH)
        log_queue.put({"type": "LOG", "src": "MUX", "msg": "stopped"})


if __name__ == "__main__":
    args = parse_args()
    
    # Simple print wrapper for standalone
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
