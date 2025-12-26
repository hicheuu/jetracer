#!/usr/bin/env python3
import multiprocessing
import time
import argparse
import sys
import os

from jetracer.mux.mux import run_mux, SOCK_PATH
from jetracer.mux.joystick import run_joystick
from jetracer.mux.udp_recv import run_udp
import socket
import json

# OS 호환성을 위한 키보드 입력 핸들러
if os.name == 'nt':
    import msvcrt
    def get_key():
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch in [b'\xe0', b'\x00']:
                sub = msvcrt.getch()
                if sub == b'H': return 'UP'
                if sub == b'P': return 'DOWN'
            return ch.decode() if isinstance(ch, bytes) else ch
        return None
else:
    import select
    import termios
    import tty
    def get_key():
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            old_settings = termios.tcgetattr(sys.stdin)
            try:
                tty.setcbreak(sys.stdin.fileno())
                ch = sys.stdin.read(1)
                if ch == '\x1b':
                    extra = sys.stdin.read(2)
                    if extra == '[A': return 'UP'
                    if extra == '[B': return 'DOWN'
                return ch
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        return None

def runner(args):
    """
    MUX, Joystick, UDP 수신 프로세스를 통합 실행하고 로그를 관리합니다.
    """
    # 모든 프로세스의 로그를 수집할 큐
    log_queue = multiprocessing.Queue()
    stop_event = multiprocessing.Event()

    # 1. MUX 프로세스 (중심 제어 루프)
    # 이제 speed5-throttle 설정은 물리적 매핑을 담당하는 MUX가 가져갑니다.
    p_mux = multiprocessing.Process(
        target=run_mux, 
        args=(log_queue, stop_event, args.speed5_throttle)
    )
    p_mux.start()

    # 2. Joystick 프로세스
    p_joy = multiprocessing.Process(
        target=run_joystick, 
        args=(log_queue, stop_event),
        kwargs={
            "device": args.device,
            "max_throttle": args.max_throttle, # 수동 제어 시의 최대 출력 제한
            "steer_scale": args.steer_scale
        }
    )
    p_joy.start()

    # 3. UDP 수신 프로세스
    # 이제 UDP는 단순히 속도 값을 전달하므로 speed5-throttle 인자가 필요 없습니다.
    p_udp = multiprocessing.Process(
        target=run_udp,
        args=(log_queue, stop_event)
    )
    p_udp.start()

    print("[RUNNER] 모든 제어 프로세스가 시작되었습니다.")
    print("[RUNNER] {Up Arrow}: Throttle +0.001 | {Down Arrow}: Throttle -0.001")
    print("[RUNNER] Ctrl+C를 눌러 종료하세요.")
    
    current_mode = "joystick" # 초기 기본 모드
    speed5_phys = args.speed5_throttle

    # MUX 전송용 유닉스 소켓
    runner_sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    try:
        while True:
            # 1. 키보드 입력 처리
            key = get_key()
            if key:
                changed = False
                if key == 'UP': # Up Arrow
                    speed5_phys += 0.001
                    changed = True
                elif key == 'DOWN': # Down Arrow
                    speed5_phys -= 0.001
                    changed = True
                
                if changed:
                    try:
                        # MUX에 업데이트 메시지 전송
                        update_msg = {
                            "src": "runner",
                            "event": "update_speed5",
                            "val": speed5_phys
                        }
                        runner_sock.sendto(json.dumps(update_msg).encode(), SOCK_PATH)
                        print(f"\r[RUNNER] SPEED5_THROTTLE UPDATED: {speed5_phys:.3f}", end="")
                    except Exception as e:
                        print(f"\n[RUNNER] Send error: {e}")

            # 2. 로그 큐 비우기 (최대한 빨리 처리하여 렉 유발 방지)
            while not log_queue.empty():
                try:
                    record = log_queue.get_nowait()
                    
                    if record.get("type") == "MODE":
                        current_mode = record["mode"]
                        continue

                    if record.get("type") == "LOG":
                        src = record["src"]
                        msg = record["msg"]
                        
                        should_print = False
                        if src == "MUX":
                            should_print = True
                        elif src == "JOY":
                            if current_mode == "joystick":
                                should_print = True
                            elif any(k in msg for k in ["Error", "Device", "stopping"]):
                                should_print = True
                        elif src == "UDP":
                            if current_mode == "udp":
                                should_print = True
                            elif "Error" in msg or "stopping" in msg:
                                should_print = True

                        if should_print:
                            # \r를 사용하여 업데이트 로그와 겹치지 않게 처리
                            print(f"\n[{src}] {msg}")

                except Exception:
                    break
            
            # 메인 프로세스 생존 확인
            if not p_mux.is_alive():
                print("\n[RUNNER] MUX 프로세스가 예기치 않게 종료되었습니다.")
                break
                
            time.sleep(0.01) # CPU 점유율 조절
                
    except KeyboardInterrupt:
        print("\n[RUNNER] 종료 중...")
    finally:
        # 모든 프로세스 종료 요청
        stop_event.set()
        p_mux.join(timeout=1)
        p_joy.join(timeout=1)
        p_udp.join(timeout=1)
        
        # 종료되지 않은 프로세스 강제 종료
        if p_mux.is_alive(): p_mux.terminate()
        if p_joy.is_alive(): p_joy.terminate()
        if p_udp.is_alive(): p_udp.terminate()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Jetracer Unified Runner (Normalized Control)")
    
    # 공통 제어 인자
    parser.add_argument("--speed5-throttle", type=float, default=0.20, help="UDP 속도 5.0일 때의 물리적 스로틀 값")
    parser.add_argument("--device", default=None, help="조이스틱 장치 경로 (예: /dev/input/event2)")
    parser.add_argument("--max-throttle", type=float, default=0.30, help="조이스틱 제어 시 최대 스로틀 제한 (0.0 ~ 1.0)")
    parser.add_argument("--steer-scale", type=float, default=1.0, help="조이스틱 조향 배율")
    
    args = parser.parse_args()
    
    runner(args)
