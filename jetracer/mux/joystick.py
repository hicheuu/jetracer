#!/usr/bin/env python3
import argparse
import time
import json
import socket
import threading

from evdev import InputDevice, ecodes

# ======================
# UDS 설정
# ======================
SOCK_PATH = "/tmp/jetracer_ctrl.sock"
udsock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

# ======================
# Xbox 360 매핑 (확정)
# ======================
STEER_AXIS = ecodes.ABS_X
THROTTLE_AXIS = ecodes.ABS_RY

TOGGLE_BTN = ecodes.BTN_Y
STOP_BTN   = ecodes.BTN_X


def clamp(x, lo=-1.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x


def apply_deadzone(v, dz):
    return 0.0 if abs(v) < dz else v


def norm_axis(val, lo, hi):
    return (2.0 * (val - lo) / (hi - lo)) - 1.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/input/event2")
    ap.add_argument("--deadzone", type=float, default=0.08)
    ap.add_argument("--steer-scale", type=float, default=1.0)
    ap.add_argument("--throttle-scale", type=float, default=0.24)
    ap.add_argument("--invert-steer", action="store_true")
    ap.add_argument("--invert-throttle", action="store_true", default=True)
    ap.add_argument("--hz", type=float, default=30.0)
    args = ap.parse_args()

    dev = InputDevice(args.device)
    print(f"[JOY] Using device: {dev.path} ({dev.name})")

    # ===== 공유 상태 =====
    steer_raw = 0.0
    thr_raw = 0.0

    steer_cmd = 0.0
    throttle_cmd = 0.12

    last_toggle = 0
    last_stop = 0

    ESC_NEUTRAL = 0.12
    REVERSE_START = -0.1

    lock = threading.Lock()
    period = 1.0 / args.hz

    # ======================
    # 송신 스레드 (고정 주기)
    # ======================
    def sender():
        nonlocal steer_cmd, throttle_cmd
        while True:
            with lock:
                msg = {
                    "src": "joystick",
                    "steer": steer_cmd,
                    "throttle": throttle_cmd
                }
            udsock.sendto(json.dumps(msg).encode(), SOCK_PATH)
            time.sleep(period)

    threading.Thread(target=sender, daemon=True).start()

    # ======================
    # evdev 이벤트 루프
    # ======================
    try:
        for event in dev.read_loop():
            with lock:
                if event.type == ecodes.EV_KEY:
                    if event.code == TOGGLE_BTN:
                        if event.value == 1 and last_toggle == 0:
                            udsock.sendto(
                                json.dumps({"src": "joystick", "event": "toggle"}).encode(),
                                SOCK_PATH
                            )
                            print("[BTN] toggle")
                        last_toggle = event.value

                    if event.code == STOP_BTN:
                        if event.value == 1 and last_stop == 0:
                            udsock.sendto(
                                json.dumps({"src": "joystick", "event": "estop"}).encode(),
                                SOCK_PATH
                            )
                            print("[BTN] EMERGENCY STOP")
                        last_stop = event.value

                elif event.type == ecodes.EV_ABS:
                    if event.code == STEER_AXIS:
                        steer_raw = norm_axis(event.value, -32768, 32767)
                    elif event.code == THROTTLE_AXIS:
                        thr_raw = norm_axis(event.value, -32768, 32767)

                # ===== 제어 계산 =====
                steer = apply_deadzone(steer_raw, args.deadzone)
                thr = apply_deadzone(thr_raw, args.deadzone)

                if args.invert_steer:
                    steer = -steer
                if args.invert_throttle:
                    thr = -thr

                if thr > 0:
                    throttle_cmd = ESC_NEUTRAL + thr * (1.0 - ESC_NEUTRAL) * args.throttle_scale
                elif thr < 0:
                    throttle_cmd = REVERSE_START + thr * (1.0 - abs(REVERSE_START)) * args.throttle_scale
                else:
                    throttle_cmd = ESC_NEUTRAL

                throttle_cmd = clamp(throttle_cmd, -1.0, 1.0)
                steer_cmd = clamp(steer * args.steer_scale)

    except KeyboardInterrupt:
        print("\n[JOY] stopping")
    finally:
        udsock.close()


if __name__ == "__main__":
    main()
