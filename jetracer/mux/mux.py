#!/usr/bin/env python3
import socket
import json
import time
import os

from jetracer.core import NvidiaRacecar

SOCK_PATH = "/tmp/jetracer_ctrl.sock"

JOY_TIMEOUT = 0.5
UDP_TIMEOUT = 1.2   # ← UDP watchdog보다 살짝 큼
ESC_NEUTRAL = 0.12

# 소켓 초기화
if os.path.exists(SOCK_PATH):
    os.unlink(SOCK_PATH)

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

print("[MUX] started | mode=joystick")

try:
    while True:
        # ===== 수신 =====
        try:
            data, _ = sock.recvfrom(512)
            msg = json.loads(data.decode())
            src = msg.get("src")

            # 이벤트
            if msg.get("event") == "toggle":
                mode = "udp" if mode == "joystick" else "joystick"
                print(f"[MUX] MODE → {mode}")
                continue

            if msg.get("event") == "estop":
                estop = not estop
                print(f"[MUX] ESTOP {'ON' if estop else 'OFF'}")
                continue

            # 명령
            msg["ts"] = time.time()
            if src == "joystick":
                last_joy = msg
            elif src == "udp":
                last_udp = msg

        except BlockingIOError:
            pass

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
            print(
                f"[MUX] {src_used} "
                f"steer={cmd['steer']:+.3f} thr={cmd['throttle']:.3f}"
            )
        else:
            car.steering = 0.0
            car.throttle = ESC_NEUTRAL

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n[MUX] stopping")
finally:
    car.steering = 0.0
    car.throttle = ESC_NEUTRAL
    sock.close()
    if os.path.exists(SOCK_PATH):
        os.unlink(SOCK_PATH)
