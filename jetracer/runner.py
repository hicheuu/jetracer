#!/usr/bin/env python3
import multiprocessing
import time
import argparse
import sys
import os

from jetracer.mux.mux import run_mux
from jetracer.mux.joystick import run_joystick
from jetracer.mux.udp_recv import run_udp

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

    print("[RUNNER] 모든 제어 프로세스가 시작되었습니다. Ctrl+C를 눌러 종료하세요.")
    
    current_mode = "joystick" # 초기 기본 모드

    try:
        while True:
            try:
                # 0.1초 대기하며 로그 큐 확인
                record = log_queue.get(timeout=0.1)
                
                # 모드 전환 이벤트 처리
                if record.get("type") == "MODE":
                    current_mode = record["mode"]
                    continue

                # 로그 출력 (현재 선택된 모드에 따라 필터링)
                if record.get("type") == "LOG":
                    src = record["src"]
                    msg = record["msg"]
                    
                    should_print = False
                    
                    # MUX 로그는 항상 출력
                    if src == "MUX":
                        should_print = True
                        
                    # 현재 활성화된 입력 소스의 로그만 출력 (에러 제외)
                    elif src == "JOY":
                        if current_mode == "joystick":
                            should_print = True
                        elif "Error" in msg or "Device" in msg or "stopping" in msg:
                            should_print = True

                    elif src == "UDP":
                        if current_mode == "udp":
                            should_print = True
                        elif "Error" in msg or "stopping" in msg:
                            should_print = True

                    if should_print:
                        print(f"[{src}] {msg}")

            except multiprocessing.queues.Empty:
                pass
            
            # 메인 프로세스 생존 확인
            if not p_mux.is_alive():
                print("[RUNNER] MUX 프로세스가 예기치 않게 종료되었습니다.")
                break
                
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
