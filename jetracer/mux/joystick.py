#!/usr/bin/env python3
import argparse
import time
import json
import socket

import pygame

# ======================
# UDS 설정
# ======================
SOCK_PATH = "/tmp/jetracer_ctrl.sock"
udsock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

TOGGLE_BTN = 5
STOP_BTN   = 4


def clamp(x, lo=-1.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x


def apply_deadzone(v, dz):
    return 0.0 if abs(v) < dz else v


def main():
    ap = argparse.ArgumentParser(description="Joystick -> JetRacer (UDS sender)")
    ap.add_argument("--steer-axis", type=int, default=0)
    ap.add_argument("--throttle-axis", type=int, default=4)
    ap.add_argument("--invert-steer", action="store_true")
    ap.add_argument("--invert-throttle", action="store_true", default=True)
    ap.add_argument("--deadzone", type=float, default=0.08)
    ap.add_argument("--steer-scale", type=float, default=1.0)
    ap.add_argument("--throttle-scale", type=float, default=0.125)
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No joystick detected")

    js = pygame.joystick.Joystick(0)
    js.init()

    last_toggle = 0
    last_stop = 0

    try:
        while True:
            pygame.event.pump()

            # ===== 버튼 처리 =====
            toggle = js.get_button(TOGGLE_BTN)
            stop = js.get_button(STOP_BTN)

            if toggle == 1 and last_toggle == 0:
                udsock.sendto(
                    json.dumps({"src": "joystick", "event": "toggle"}).encode(),
                    SOCK_PATH
                )
                print("[BTN] toggle")

            if stop == 1 and last_stop == 0:
                udsock.sendto(
                    json.dumps({"src": "joystick", "event": "estop"}).encode(),
                    SOCK_PATH
                )
                print("[BTN] EMERGENCY STOP")

            last_toggle = toggle
            last_stop = stop

            # ===== 축 처리 (기존 로직 유지) =====
            steer_raw = js.get_axis(args.steer_axis)
            thr_raw = js.get_axis(args.throttle_axis)

            steer = apply_deadzone(steer_raw, args.deadzone)
            thr = apply_deadzone(thr_raw, args.deadzone)

            if args.invert_steer:
                steer = -steer
            if args.invert_throttle:
                thr = -thr

            ESC_NEUTRAL = 0.12
            REVERSE_START = -0.1

            if thr > 0:
                throttle_cmd = ESC_NEUTRAL + thr * (1.0 - ESC_NEUTRAL) * args.throttle_scale
            elif thr < 0:
                throttle_cmd = REVERSE_START + thr * (1.0 - abs(REVERSE_START)) * args.throttle_scale
            else:
                throttle_cmd = ESC_NEUTRAL

            throttle_cmd = clamp(throttle_cmd, -1.0, 1.0)
            steering_cmd = clamp(steer * args.steer_scale)

            # ===== UDS 송신 =====
            udsock.sendto(
                json.dumps({
                    "src": "joystick",
                    "steer": steering_cmd,
                    "throttle": throttle_cmd
                }).encode(),
                SOCK_PATH
            )

            time.sleep(0.03)

    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
