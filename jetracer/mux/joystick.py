import argparse
import time
import json
import socket
import threading
from evdev import InputDevice, ecodes, list_devices
import multiprocessing

# ======================
# UDS 설정
# ======================
SOCK_PATH = "/tmp/jetracer_ctrl.sock"

# ======================
# Xbox 360 매핑 정의
# ======================
STEER_AXIS = ecodes.ABS_X
THROTTLE_AXIS = ecodes.ABS_RY

TOGGLE_BTN = ecodes.BTN_Y
STOP_BTN   = ecodes.BTN_X

THR_UP_BTN   = ecodes.BTN_TR   # RB
THR_DOWN_BTN = ecodes.BTN_TL   # LB


def clamp(x, lo=-1.0, hi=1.0):
    """
    주어진 값을 가용 범위 내로 제한합니다.
    """
    return lo if x < lo else hi if x > hi else x


def apply_deadzone(v, dz):
    """
    입력 신호에 데드존을 적용하여 미세한 노이즈를 제거합니다.
    """
    return 0.0 if abs(v) < dz else v


def norm_axis(val, lo, hi):
    """
    입력 축 값을 -1.0에서 1.0 범위로 정규화합니다.
    """
    return (2.0 * (val - lo) / (hi - lo)) - 1.0


def find_device(target_name: str = None):
    """
    입력 장치를 검색하여 경로를 반환합니다.
    """
    devices = [InputDevice(path) for path in list_devices()]
    if not devices:
        return None

    keywords = ["Xbox", "Gamepad", "Controller"]
    if target_name:
        keywords = [target_name]

    for kw in keywords:
        for dev in devices:
            if kw.lower() in dev.name.lower():
                return dev.path
    
    return None


def run_joystick(log_queue, stop_event, device=None, deadzone=0.08, steer_scale=1.0, max_throttle=0.3, invert_steer=False, invert_throttle=True, hz=30.0):
    """
    조이스틱 입력을 처리하고 MUX에 정규화된 제어 메시지를 송신하는 메인 함수입니다.
    """
    udsock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    
    log_queue.put({"type": "LOG", "src": "JOY", "msg": "Started with Normalized Control Architecture"})

    # 장치 자동 감지
    device_path = device
    if device_path is None:
        detected = find_device()
        if detected:
            device_path = detected
        else:
            device_path = "/dev/input/event2"
            log_queue.put({"type": "LOG", "src": "JOY", "msg": f"Auto-detected failed. Using fallback: {device_path}"})

    try:
        dev = InputDevice(device_path)
        log_queue.put({"type": "LOG", "src": "JOY", "msg": f"Using device: {dev.path} ({dev.name})"})
    except Exception as e:
        log_queue.put({"type": "LOG", "src": "JOY", "msg": f"Error opening device: {e}"})
        return

    # 상태 변수 초기화
    steer_cmd = 0.0
    throttle_cmd = 0.0
    current_max_throttle = max_throttle

    last_toggle = 0
    last_toggle_time = 0.0
    TOGGLE_DEBOUNCE = 0.5
    
    last_stop = 0
    last_thr_up = 0
    last_thr_dn = 0

    lock = threading.Lock()
    period = 1.0 / hz
    running = True

    # 전송용 백그라운드 스레드
    def sender_loop():
        """
        일정한 주기로 MUX에 최신 제어 명령을 전송합니다.
        """
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

    # 조이스틱 이벤트 루프
    try:
        while not stop_event.is_set():
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
                    # ---------- 버튼 처리 ----------
                    if event.type == ecodes.EV_KEY:
                        if event.code == TOGGLE_BTN:
                            now_btn = time.time()
                            if event.value == 1 and last_toggle == 0 and (now_btn - last_toggle_time) > TOGGLE_DEBOUNCE:
                                udsock.sendto(
                                    json.dumps({"src": "joystick", "event": "toggle"}).encode(),
                                    SOCK_PATH
                                )
                                log_queue.put({"type": "LOG", "src": "JOY", "msg": "[BTN] mode toggle"})
                                last_toggle_time = now_btn
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
                                log_queue.put({"type": "LOG", "src": "JOY", "msg": f"[THR] max limit set to {current_max_throttle:.2f}"})
                            last_thr_up = event.value

                        elif event.code == THR_DOWN_BTN:
                            if event.value == 1 and last_thr_dn == 0:
                                current_max_throttle = clamp(current_max_throttle - 0.01, 0.0, 1.0)
                                log_queue.put({"type": "LOG", "src": "JOY", "msg": f"[THR] max limit set to {current_max_throttle:.2f}"})
                            last_thr_dn = event.value

                    # ---------- 축(Analog Stick) 처리 ----------
                    elif event.type == ecodes.EV_ABS:
                        if event.code == STEER_AXIS:
                            raw_norm = norm_axis(event.value, -32768, 32767)
                            val = apply_deadzone(raw_norm, deadzone)
                            if invert_steer: val = -val
                            steer_cmd = clamp(val * steer_scale)

                        elif event.code == THROTTLE_AXIS:
                            raw_norm = norm_axis(event.value, -32768, 32767)
                            val = apply_deadzone(raw_norm, deadzone)
                            if invert_throttle: val = -val
                            # 정규화된 출력 산출 (-1.0 ~ 1.0)
                            throttle_cmd = clamp(val * current_max_throttle)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        log_queue.put({"type": "LOG", "src": "JOY", "msg": f"Exception: {e}"})
    finally:
        running = False
        log_queue.put({"type": "LOG", "src": "JOY", "msg": "stopping"})
        time.sleep(period * 1.5)
        udsock.close()


def main():
    """
    모듈 독립 실행 시의 엔트리 포인트입니다.
    """
    ap = argparse.ArgumentParser(description="evdev joystick (Normalized Output)")
    ap.add_argument("--device", default=None)
    ap.add_argument("--deadzone", type=float, default=0.08)
    ap.add_argument("--steer-scale", type=float, default=1.0)
    ap.add_argument("--max-throttle", type=float, default=0.3)
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
        run_joystick(PrintQueue(), stop, **vars(args))
    except KeyboardInterrupt:
        stop.set()

if __name__ == "__main__":
    main()
