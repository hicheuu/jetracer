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


def main():
    ap = argparse.ArgumentParser(description="evdev joystick (direct throttle limit)")
    ap.add_argument("--device", default="/dev/input/event2")
    ap.add_argument("--deadzone", type=float, default=0.08)
    ap.add_argument("--steer-scale", type=float, default=1.0)
    ap.add_argument("--max-throttle", type=float, default=0.24)
    ap.add_argument("--invert-steer", action="store_true")
    ap.add_argument("--invert-throttle", action="store_true", default=True)
    ap.add_argument("--hz", type=float, default=30.0)
    args = ap.parse_args()

    dev = InputDevice(args.device)
    print(f"[JOY] Using device: {dev.path} ({dev.name})")

    # ======================
    # 상태 변수
    # ======================
    steer = 0.0
    thr = 0.0

    steer_cmd = 0.0
    throttle_cmd = 0.12

    max_throttle = args.max_throttle

    last_toggle = 0
    last_stop = 0
    last_thr_up = 0
    last_thr_dn = 0

    ESC_NEUTRAL = 0.12
    REVERSE_START = -0.1

    lock = threading.Lock()
    period = 1.0 / args.hz
    running = True

    # ======================
    # 송신 스레드
    # ======================
    def sender_loop():
        while running:
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

    threading.Thread(target=sender_loop, daemon=True).start()

    # ======================
    # evdev 이벤트 루프
    # ======================
    try:
        for event in dev.read_loop():
            with lock:
                # ---------- 버튼 ----------
                if event.type == ecodes.EV_KEY:
                    if event.code == TOGGLE_BTN:
                        if event.value == 1 and last_toggle == 0:
                            udsock.sendto(
                                json.dumps({"src": "joystick", "event": "toggle"}).encode(),
                                SOCK_PATH
                            )
                            print("[BTN] toggle")
                        last_toggle = event.value

                    elif event.code == STOP_BTN:
                        if event.value == 1 and last_stop == 0:
                            udsock.sendto(
                                json.dumps({"src": "joystick", "event": "estop"}).encode(),
                                SOCK_PATH
                            )
                            print("[BTN] EMERGENCY STOP")
                        last_stop = event.value

                    elif event.code == THR_UP_BTN:
                        if event.value == 1 and last_thr_up == 0:
                            max_throttle = clamp(max_throttle + 0.01, 0.0, 1.0)
                            print(f"[THR] max_throttle ↑ {max_throttle:.2f}")
                        last_thr_up = event.value

                    elif event.code == THR_DOWN_BTN:
                        if event.value == 1 and last_thr_dn == 0:
                            max_throttle = clamp(max_throttle - 0.01, 0.0, 1.0)
                            print(f"[THR] max_throttle ↓ {max_throttle:.2f}")
                        last_thr_dn = event.value

                # ---------- 축 ----------
                elif event.type == ecodes.EV_ABS:
                    if event.code == STEER_AXIS:
                        steer = apply_deadzone(
                            norm_axis(event.value, -32768, 32767),
                            args.deadzone
                        )
                        if args.invert_steer:
                            steer = -steer

                    elif event.code == THROTTLE_AXIS:
                        thr = apply_deadzone(
                            norm_axis(event.value, -32768, 32767),
                            args.deadzone
                        )
                        if args.invert_throttle:
                            thr = -thr

                # ---------- 최종 명령 ----------
                if thr > 0:
                    throttle_cmd = ESC_NEUTRAL + thr * max_throttle
                elif thr < 0:
                    throttle_cmd = REVERSE_START + thr * abs(REVERSE_START)
                else:
                    throttle_cmd = ESC_NEUTRAL

                throttle_cmd = clamp(throttle_cmd, -1.0, 1.0)
                steer_cmd = clamp(steer * args.steer_scale)

    except KeyboardInterrupt:
        print("\n[JOY] stopping")
    finally:
        running = False
        time.sleep(period * 1.5)
        udsock.close()


if __name__ == "__main__":
    main()
