import socket
import json
import time
import os
import sys
import multiprocessing

from jetracer.core import NvidiaRacecar

SOCK_PATH = "/tmp/jetracer_ctrl.sock"

JOY_TIMEOUT = 0.5
UDP_TIMEOUT = 1.2   # ← UDP watchdog보다 살짝 큼

def load_config():
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(cur_dir, "../../config/nvidia_racecar_config.json")
    
    try:
        with open(config_path, "r") as f:
            config = json.load(f)
            neutral = config.get("throttle", {}).get("neutral")
            if neutral is None:
                raise ValueError("Output throttle.neutral not found in config")
            print(f"[MUX] Loaded ESC_NEUTRAL={neutral} from config")
            return neutral
    except Exception as e:
        print(f"[MUX] CRITICAL: Failed to load config: {e}")
        sys.exit(1)

def run_mux(log_queue, stop_event):
    ESC_NEUTRAL = load_config()

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
                car.throttle = cmd["throttle"]
                
                # Log throttling (0.5s)
                if now - last_log_time > 0.5:
                    log_queue.put({
                        "type": "LOG", 
                        "src": "MUX", 
                        "msg": f"{src_used} steer={cmd['steer']:+.3f} thr={cmd['throttle']:.3f}"
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
    # Standalone support
    import queue
    q = queue.Queue()
    stop = multiprocessing.Event()
    
    # Simple print wrapper for standalone
    class PrintQueue:
        def put(self, item):
            if item.get("type") == "LOG":
                print(f"[{item['src']}] {item['msg']}")
            elif item.get("type") == "MODE":
                 print(f"[{item['mode']}] MODE CHANGE")

    try:
        run_mux(PrintQueue(), stop)
    except KeyboardInterrupt:
        stop.set()
