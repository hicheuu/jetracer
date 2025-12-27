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
    p_mux = multiprocessing.Process(
        target=run_mux, 
        args=(log_queue, stop_event, args.speed5_throttle, args.log_calibration)
    )
    p_mux.start()

    # 2. Joystick 프로세스
    p_joy = multiprocessing.Process(
        target=run_joystick, 
        args=(log_queue, stop_event),
        kwargs={
            "device": args.device,
            "steer_scale": args.steer_scale
        }
    )
    p_joy.start()

    # 3. UDP 수신 프로세스
    p_udp = multiprocessing.Process(
        target=run_udp,
        args=(log_queue, stop_event, args.auto_calibrate, args.target_velocity),
        kwargs={
            "window_duration": args.auto_calibrate_window,
            "increment": args.auto_calibrate_increment,
            "threshold": args.auto_calibrate_threshold
        }
    )
    p_udp.start()

    print("[RUNNER] 모든 제어 프로세스가 시작되었습니다.")
    print("[RUNNER] 조이스틱 RB/LB 버튼으로 SPEED5 ±0.001 조절")
    print("[RUNNER] Ctrl+C를 눌러 종료하세요.")
    
    current_mode = "joystick" # 초기 기본 모드

    try:
        while True:
            # 로그 큐 비우기 (최대한 빨리 처리하여 렉 유발 방지)
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
                            # UDP 로그: 자동보정 요약, 에러, 종료 메시지는 항상 출력
                            if any(k in msg for k in ["Auto-Calib", "Error", "stopping", "종료"]):
                                should_print = True
                            elif current_mode == "udp" and not args.quiet_udp:
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
    parser.add_argument("--speed5-throttle", type=float, default=0.20, help="초기 물리적 스로틀 목표값 (조이스틱 RB/LB로 0.001 단위 조절 가능)")
    parser.add_argument("--device", default=None, help="조이스틱 장치 경로 (예: /dev/input/event2)")
    parser.add_argument("--steer-scale", type=float, default=1.0, help="조이스틱 조향 배율")
    parser.add_argument("--log-calibration", action="store_true", help="속도 캘리브레이션용 데이터 로깅 활성화")
    parser.add_argument("--auto-calibrate", action="store_true", help="실시간 실제 속도 기반 스로틀 자동 보정 활성화")
    parser.add_argument("--target-velocity", type=float, default=5.0, help="자동 보정 시 목표로 하는 실제 차량 속도 (m/s)")
    parser.add_argument("--auto-calibrate-window", type=float, default=3.0, help="자동 보정 시 평균 속도를 계산할 윈도우 시간 (초)")
    parser.add_argument("--auto-calibrate-increment", type=float, default=0.005, help="자동 보정 시 한 번에 조절할 스로틀 양")
    parser.add_argument("--auto-calibrate-threshold", type=float, default=4.5, help="자동 보정이 트리거되는 평균 속도 임계값 (m/s)")
    parser.add_argument("--quiet-udp", action="store_true", help="UDP 모드 루틴 로그 숨기기 (에러/자동보정 요약은 표시)")
    
    args = parser.parse_args()
    
    runner(args)
