#!/usr/bin/env python3
import argparse
import time
import pygame
from jetracer.nvidia_racecar import NvidiaRacecar

def clamp(x, lo=-1.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x

def apply_deadzone(v, dz):
    return 0.0 if abs(v) < dz else v

def main():
    ap = argparse.ArgumentParser(description="Joystick -> JetRacer (pure Python)")
    ap.add_argument("--steer-axis", type=int, default=0, help="steering axis index (default: 0)")
    ap.add_argument("--throttle-axis", type=int, default=1, help="throttle axis index (default: 1)")
    ap.add_argument("--invert-steer", action="store_true", help="invert steering")
    ap.add_argument("--invert-throttle", action="store_true", help="invert throttle")
    ap.add_argument("--deadzone", type=float, default=0.08, help="deadzone for axes")
    ap.add_argument("--throttle-mode", choices=["stick","trigger"], default="stick",
                    help="stick: -1..1 -> 0..1 매핑 / trigger: 0..1 가정")
    ap.add_argument("--steer-scale", type=float, default=1.0, help="steering scale (0~1)")
    ap.add_argument("--throttle-scale", type=float, default=0.3, help="throttle scale (0~1)")
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No joystick detected!")

    js = pygame.joystick.Joystick(0)
    js.init()
    n_axes = js.get_numaxes()
    print(f"Joystick: {js.get_name()} | axes={n_axes} buttons={js.get_numbuttons()}")
    if args.steer_axis >= n_axes or args.throttle_axis >= n_axes:
        raise RuntimeError(f"축 개수({n_axes})보다 큰 인덱스가 지정됨: "
                           f"steer={args.steer_axis}, throttle={args.throttle_axis}")

    car = NvidiaRacecar()
    car.steering = 0.0
    car.throttle = 0.0

    try:
        while True:
            pygame.event.pump()

            steer_raw = js.get_axis(args.steer_axis)
            thr_raw = js.get_axis(args.throttle_axis)

            # deadzone
            steer = apply_deadzone(steer_raw, args.deadzone)
            thr = apply_deadzone(thr_raw, args.deadzone)

            # invert
            if args.invert_steer:
                steer = -steer
            if args.invert_throttle:
                thr = -thr

            # throttle mapping
            if args.throttle_mode == "stick":
                # -1..1  ->  0..1
                throttle = (thr + 1.0) / 2.0
            else:  # "trigger"
                # 0..1 가정 (컨트롤러에 따라 -1..1 일 수도 있음)
                # 만약 -1..1 로 나오면 위 probe로 확인 후 --throttle-mode stick 사용
                throttle = thr

            # scaling & clamp
            steering_cmd = clamp(steer * args.steer_scale)
            throttle_cmd = max(0.0, min(1.0, throttle * args.throttle_scale))

            car.steering = steering_cmd
            car.throttle = throttle_cmd

            print(f"steer_axis[{args.steer_axis}]={steer_raw:+.2f} -> {steering_cmd:+.2f} | "
                  f"thr_axis[{args.throttle_axis}]={thr_raw:+.2f} -> {throttle_cmd:.2f}")
            time.sleep(0.03)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        car.steering = 0.0
        car.throttle = 0.0
        pygame.quit()

if __name__ == "__main__":
    main()

