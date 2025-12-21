#!/usr/bin/env python3
import argparse
import time

import pygame

from jetracer.core import NvidiaRacecar


def clamp(x, lo=-1.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x


def apply_deadzone(v, dz):
    return 0.0 if abs(v) < dz else v


def main():
    ap = argparse.ArgumentParser(description="Joystick -> JetRacer (pure Python)")
    ap.add_argument("--steer-axis", type=int, default=0, help="steering axis index (default: 0, 왼쪽스틱 좌우)")
    ap.add_argument("--throttle-axis", type=int, default=4, help="throttle axis index (default: 3, 오른쪽스틱 위아래)")
    ap.add_argument("--invert-steer", action="store_true", help="invert steering")
    ap.add_argument("--invert-throttle", action="store_true", default=True, help="invert throttle (기본 활성화)")
    ap.add_argument("--deadzone", type=float, default=0.08, help="deadzone for axes")
    ap.add_argument("--throttle-mode", choices=["stick", "trigger"], default="stick", help="stick: -1..1 -> 0..1 매핑 / trigger: 0..1 가정")
    ap.add_argument("--steer-scale", type=float, default=1.0, help="steering scale (0~1)")
    ap.add_argument("--throttle-scale", type=float, default=0.125, help="throttle scale (0~1), 기본 최대 ~0.45 m/s")
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
        raise RuntimeError(
            f"축 개수({n_axes})보다 큰 인덱스가 지정됨: "
            f"steer={args.steer_axis}, throttle={args.throttle_axis}"
        )

    car = NvidiaRacecar()
    car.steering = 0.0
    car.throttle = 0.12  # ESC 중립점

    try:
        while True:
            pygame.event.pump()

            steer_raw = js.get_axis(args.steer_axis)
            thr_raw = js.get_axis(args.throttle_axis)

            steer = apply_deadzone(steer_raw, args.deadzone)
            thr = apply_deadzone(thr_raw, args.deadzone)

            if args.invert_steer:
                steer = -steer
            if args.invert_throttle:
                thr = -thr

            # ESC 중립점 및 후진 시작점 설정
            ESC_NEUTRAL = 0.12  # 정지 상태
            REVERSE_START = -0.1  # 후진 시작점
            
            if thr > 0:
                # 전진: 0 → 0.12, 1 → 1.0
                throttle_cmd = ESC_NEUTRAL + thr * (1.0 - ESC_NEUTRAL) * args.throttle_scale
            elif thr < 0:
                # 후진: 0 → -0.1, -1 → -1.0
                throttle_cmd = REVERSE_START + thr * (1.0 - abs(REVERSE_START)) * args.throttle_scale
            else:
                throttle_cmd = ESC_NEUTRAL  # 정지
            
            # 안전 범위 클리핑
            throttle_cmd = max(-1.0, min(1.0, throttle_cmd))
            
            steering_cmd = clamp(steer * args.steer_scale)

            car.steering = steering_cmd
            car.throttle = throttle_cmd

            print(
                f"steer_axis[{args.steer_axis}]={steer_raw:+.2f} -> {steering_cmd:+.2f} | "
                f"thr_axis[{args.throttle_axis}]={thr_raw:+.4f} -> {throttle_cmd:.4f}"
            )
            time.sleep(0.03)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        car.steering = 0.0
        car.throttle = 0.12  # ESC 중립점
        pygame.quit()


if __name__ == "__main__":
    main()

