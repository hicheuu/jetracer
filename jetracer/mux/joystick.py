import argparse
import time
import json
import socket
import threading
import sys
from evdev import InputDevice, ecodes, list_devices
import os
import multiprocessing

from jetracer.core.nvidia_racecar import load_config

# ======================
# UDS 설정
# ======================
SOCK_PATH = "/tmp/jetracer_ctrl.sock"
udsock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

# ======================
# Xbox 360 매핑
# ======================
STEER_AXIS = ecodes.ABS_X
THROTTLE_AXIS = ecodes.ABS_RY

TOGGLE_BTN = ecodes.BTN_Y
STOP_BTN   = ecodes.BTN_X

THR_UP_BTN   = ecodes.BTN_TR   # RB
THR_DOWN_BTN = ecodes.BTN_TL   # LB


def clamp(x, lo=-1.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x


def apply_deadzone(v, dz):
    return 0.0 if abs(v) < dz else v


def norm_axis(val, lo, hi):
    return (2.0 * (val - lo) / (hi - lo)) - 1.0


def find_device(target_name: str = None):
    """
    Available devices 중에서 이름에 target_name이 포함된 장치를 찾습니다.
    target_name이 없으면 'Xbox', 'Gamepad', 'Controller' 등의 키워드로 검색합니다.
    """
    devices = [InputDevice(path) for path in list_devices()]
    if not devices:
        return None

    # 우선순위 키워드
    keywords = ["Xbox", "Gamepad", "Controller"]
    if target_name:
        keywords = [target_name]

    for kw in keywords:
        for dev in devices:
            if kw.lower() in dev.name.lower():
                return dev.path
    
    return None



def run_joystick(log_queue, stop_event, device=None, deadzone=0.08, steer_scale=1.0, max_throttle=0.24, invert_steer=False, invert_throttle=True, hz=30.0):
    config = load_config()
    ESC_NEUTRAL = config.get("throttle", {}).get("neutral", 0.12) if config else 0.12
    log_queue.put({"type": "LOG", "src": "JOY", "msg": f"Loaded ESC_NEUTRAL={ESC_NEUTRAL} from config"})
    REVERSE_START = -0.1

    device_path = device

    # Auto-detect if device not specified
    if device_path is None:
        log_queue.put({"type": "LOG", "src": "JOY", "msg": "No device specified, attempting auto-detection..."})
        detected = find_device()
        if detected:
            device_path = detected
            log_queue.put({"type": "LOG", "src": "JOY", "msg": f"Auto-detected device: {device_path}"})
        else:
            # Fallback default
            device_path = "/dev/input/event2"
            log_queue.put({"type": "LOG", "src": "JOY", "msg": f"Check failed. Using fallback: {device_path}"})

    try:
        dev = InputDevice(device_path)
        log_queue.put({"type": "LOG", "src": "JOY", "msg": f"Using device: {dev.path} ({dev.name})"})
    except FileNotFoundError:
        log_queue.put({"type": "LOG", "src": "JOY", "msg": f"Error: Device not found at {device_path}"})
        return
    except PermissionError:
        log_queue.put({"type": "LOG", "src": "JOY", "msg": f"Error: Permission denied for {device_path}. Check udev rules or use sudo."})
        return

    # ======================
    # 상태 변수
    # ======================
    steer = 0.0
    thr = 0.0

    steer_cmd = 0.0
    throttle_cmd = 0.12

    # max_throttle variable shadows the argument, but Python allows this. 
    # To be cleaner, let's use current_max_throttle
    current_max_throttle = max_throttle

    last_toggle = 0
    last_stop = 0
    last_thr_up = 0
    last_thr_dn = 0

    lock = threading.Lock()
    period = 1.0 / hz
    running = True

    # ======================
    # 송신 스레드
    # ======================
    def sender_loop():
        while running and not stop_event.is_set():
            with lock:
                msg = {
                    "src": "joystick",
                    "steer": steer_cmd,
                    "throttle": throttle_cmd
                }
            try:
                udsock.sendto(json.dumps(msg).encode(), SOCK_PATH)
            except OSError:
                break
            time.sleep(period)

    t = threading.Thread(target=sender_loop, daemon=True)
    t.start()

    # ======================
    # evdev 이벤트 루프
    # ======================
    try:
        # We need to mix non-blocking read with stop_event check or just rely on daemon thread termination?
        # evdev read_loop is blocking. We can use select or just rely on the fact that if the main process kills us, we die.
        # But 'stop_event' implies cooperative shutdown.
        # To do cooperative shutdown with evdev, access the fd or use read_one() with timeout.
        
        while not stop_event.is_set():
            # Read events non-blocking
            events = []
            try:
                for event in dev.read():
                    events.append(event)
            except BlockingIOError:
                pass
            
            if not events:
                time.sleep(0.01)
                continue

            for event in events:
                with lock:
                    # ---------- 버튼 ----------
                    if event.type == ecodes.EV_KEY:
                        if event.code == TOGGLE_BTN:
                            if event.value == 1 and last_toggle == 0:
                                udsock.sendto(
                                    json.dumps({"src": "joystick", "event": "toggle"}).encode(),
                                    SOCK_PATH
                                )
                                log_queue.put({"type": "LOG", "src": "JOY", "msg": "[BTN] toggle"})
                            last_toggle = event.value

                        elif event.code == STOP_BTN:
                            if event.value == 1 and last_stop == 0:
                                udsock.sendto(
                                    json.dumps({"src": "joystick", "event": "estop"}).encode(),
                                    SOCK_PATH
                                )
                                log_queue.put({"type": "LOG", "src": "JOY", "msg": "[BTN] EMERGENCY STOP"})
                            last_stop = event.value

                        elif event.code == THR_UP_BTN:
                            if event.value == 1 and last_thr_up == 0:
                                current_max_throttle = clamp(current_max_throttle + 0.01, 0.0, 1.0)
                                log_queue.put({"type": "LOG", "src": "JOY", "msg": f"[THR] max_throttle ↑ {current_max_throttle:.2f}"})
                            last_thr_up = event.value

                        elif event.code == THR_DOWN_BTN:
                            if event.value == 1 and last_thr_dn == 0:
                                current_max_throttle = clamp(current_max_throttle - 0.01, 0.0, 1.0)
                                log_queue.put({"type": "LOG", "src": "JOY", "msg": f"[THR] max_throttle ↓ {current_max_throttle:.2f}"})
                            last_thr_dn = event.value

                    # ---------- 축 ----------
                    elif event.type == ecodes.EV_ABS:
                        if event.code == STEER_AXIS:
                            steer = apply_deadzone(
                                norm_axis(event.value, -32768, 32767),
                                deadzone
                            )
                            if invert_steer:
                                steer = -steer

                        elif event.code == THROTTLE_AXIS:
                            thr = apply_deadzone(
                                norm_axis(event.value, -32768, 32767),
                                deadzone
                            )
                            if invert_throttle:
                                thr = -thr

                    # ---------- 최종 명령 ----------
                    if thr > 0:
                        throttle_cmd = ESC_NEUTRAL + thr * current_max_throttle
                    elif thr < 0:
                        throttle_cmd = REVERSE_START + thr * abs(REVERSE_START)
                    else:
                        throttle_cmd = ESC_NEUTRAL

                    throttle_cmd = clamp(throttle_cmd, -1.0, 1.0)
                    steer_cmd = clamp(steer * steer_scale)

    except KeyboardInterrupt:
        pass
    except OSError:
        log_queue.put({"type": "LOG", "src": "JOY", "msg": "Device disconnected"})
    finally:
        running = False
        log_queue.put({"type": "LOG", "src": "JOY", "msg": "stopping"})
        time.sleep(period * 1.5)
        udsock.close()


def main():
    ap = argparse.ArgumentParser(description="evdev joystick (direct throttle limit)")
    ap.add_argument("--device", default=None, help="Path to input device (e.g. /dev/input/event0). If not set, auto-detect.")
    ap.add_argument("--deadzone", type=float, default=0.08)
    ap.add_argument("--steer-scale", type=float, default=1.0)
    ap.add_argument("--max-throttle", type=float, default=0.24)
    ap.add_argument("--invert-steer", action="store_true")
    ap.add_argument("--invert-throttle", action="store_true", default=True)
    ap.add_argument("--hz", type=float, default=30.0)
    args = ap.parse_args()

    class PrintQueue:
        def put(self, item):
            if item.get("type") == "LOG":
                print(f"[{item['src']}] {item['msg']}")

    stop = multiprocessing.Event()
    try:
        run_joystick(
            PrintQueue(), 
            stop,
            device=args.device,
            deadzone=args.deadzone,
            steer_scale=args.steer_scale,
            max_throttle=args.max_throttle,
            invert_steer=args.invert_steer,
            invert_throttle=args.invert_throttle,
            hz=args.hz
        )
    except KeyboardInterrupt:
        stop.set()

if __name__ == "__main__":
    main()
