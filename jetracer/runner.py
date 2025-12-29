#!/usr/bin/env python3
import multiprocessing
import time
import argparse
import sys
import os

from jetracer.mux.mux import run_mux
from jetracer.mux.joystick import run_joystick
from jetracer.mux.udp_recv import run_udp
from jetracer.teleop.udp_send_telemetry import run_telemetry
from jetracer.core.nvidia_racecar import load_config

try:
    import msvcrt
    WINDOWS_KEYBOARD = True
except ImportError:
    WINDOWS_KEYBOARD = False

def runner(args):
    """
    MUX, Joystick, UDP 수신 프로세스를 통합 실행하고 로그를 관리합니다.
    """
    # 만약 --analyze 옵션이 있다면 최소한 로깅은 켜져 있어야 분석 가능
    if getattr(args, "analyze", False):
        args.log_calibration = True

    # 모든 프로세스의 로그를 수집할 큐
    log_queue = multiprocessing.Queue()
    stop_event = multiprocessing.Event()

    # 1. MUX 프로세스 (중심 제어 루프)
    p_mux = multiprocessing.Process(
        target=run_mux, 
        args=(log_queue, stop_event, args.speed5_throttle, args.log_calibration),
        kwargs={"verbose_motor": args.log_motor}
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

    # 공유 보정 파라미터 생성 (실시간 수정 가능)
    auto_inc = multiprocessing.Value('d', args.auto_calibrate_increment)
    auto_dec = multiprocessing.Value('d', args.auto_calibrate_decrement)

    # 3. UDP 수신 프로세스
    p_udp = multiprocessing.Process(
        target=run_udp,
        args=(log_queue, stop_event, args.auto_calibrate, args.target_velocity, auto_inc, auto_dec),
        kwargs={
            "window_packets": args.auto_calibrate_packets,
            "increment": args.auto_calibrate_increment,
            "decrement": args.auto_calibrate_decrement,
            "threshold": args.auto_calibrate_threshold
        }
    )
    p_udp.start()

    # 4. Telemetry 송신 프로세스
    p_tele = multiprocessing.Process(
        target=run_telemetry,
        args=(stop_event,),
        kwargs={
            "server_ip": args.telemetry_ip,
            "hz": args.telemetry_hz,
            "car_number": args.car_number,
            "verbose": args.log_telemetry
        }
    )
    p_tele.start()

    print("[RUNNER] 모든 제어 프로세스가 시작되었습니다.")
    print("[RUNNER] 조이스틱 RB/LB 버튼으로 SPEED5 ±0.001 조절")
    print("[RUNNER] 키보드 't' : 보정 대상(Increment/Decrement) 변경")
    print("[RUNNER] 키보드 '+/-' : 현재 보정값 ±0.0001 조절")
    print("[RUNNER] Ctrl+C를 눌러 종료하세요.")
    
    current_mode = "joystick" # 초기 기본 모드
    calib_target = "increment" # "increment" or "decrement"

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
                            # UDP 로그: 자동보정 요약, 진단, 에러, 종료 메시지는 항상 출력
                            if any(k in msg for k in ["Auto-Calib", "[DIAG]", "Error", "stopping", "종료"]):
                                should_print = True
                            elif current_mode == "udp" and not args.quiet_udp:
                                should_print = True

                        if should_print:
                            # \r를 사용하여 업데이트 로그와 겹치지 않게 처리
                            print(f"\n[{src}] {msg}")

                except Exception:
                    break
            
            # 메인 프로세스 생존 확인
            for name, p in [("MUX", p_mux), ("JOY", p_joy), ("UDP", p_udp), ("TELE", p_tele)]:
                if not p.is_alive():
                    print(f"\n[RUNNER] {name} 프로세스가 예기치 않게 종료되었습니다.")
                    stop_event.set()
                    break
            
            if stop_event.is_set():
                break
                
            # 키보드 입력 처리 (Windows 전용 msvcrt)
            if WINDOWS_KEYBOARD and msvcrt.kbhit():
                char = msvcrt.getch().decode('utf-8').lower()
                if char == 't':
                    calib_target = "decrement" if calib_target == "increment" else "increment"
                    print(f"\n[RUNNER] 보정 대상 변경 -> {calib_target.upper()}")
                elif char in ['+', '=', '-', '_']:
                    delta = 0.0001 if char in ['+', '='] else -0.0001
                    if calib_target == "increment":
                        with auto_inc.get_lock():
                            auto_inc.value += delta
                        print(f"\n[RUNNER] Auto-Calib Increment: {auto_inc.value:.5f} ({delta:+.5f})")
                    else:
                        with auto_dec.get_lock():
                            auto_dec.value += delta
                        print(f"\n[RUNNER] Auto-Calib Decrement: {auto_dec.value:.5f} ({delta:+.5f})")

            time.sleep(0.01) # CPU 점유율 조절
                
    except KeyboardInterrupt:
        print("\n[RUNNER] 종료 중...")
    finally:
        # 모든 프로세스 종료 요청
        stop_event.set()
        
        # 프로세스들이 종료 로그를 보낼 시간을 주고 큐 비우기
        time.sleep(0.3)
        while not log_queue.empty():
            try:
                record = log_queue.get_nowait()
                if record.get("type") == "LOG":
                    src = record["src"]
                    msg = record["msg"]
                    print(f"[{src}] {msg}")
            except:
                break
        
        p_mux.join(timeout=1)
        p_joy.join(timeout=1)
        p_udp.join(timeout=1)
        p_tele.join(timeout=1)
        
        # 종료되지 않은 프로세스 강제 종료
        if p_mux.is_alive(): p_mux.terminate()
        if p_joy.is_alive(): p_joy.terminate()
        if p_udp.is_alive(): p_udp.terminate()
        if p_tele.is_alive(): p_tele.terminate()
        
        # 분석 도구 실행 (옵션 활성화 시)
        if getattr(args, "analyze", False):
            print("\n[RUNNER] 보정 데이터 분석을 시작합니다...")
            try:
                from jetracer.tools.calibrate_analyzer import analyze_latest_calibration
                analyze_latest_calibration()
            except ImportError as e:
                print(f"[RUNNER] Error: 분석 도구(calibrate_analyzer.py)를 불러올 수 없습니다. (사유: {e})")
                print("[RUNNER] 'pip install pandas matplotlib numpy'가 설치되어 있는지 확인하세요.")
            except Exception as e:
                print(f"[RUNNER] 분석 도구 실행 중 오류 발생: {e}")

if __name__ == "__main__":
    # 설정 파일에서 기본값 로드
    config = load_config()
    thr_config = config.get("throttle", {}) if config else {}
    
    default_inc = thr_config.get("auto_calibrate_increment", 0.001)
    default_dec = thr_config.get("auto_calibrate_decrement", -0.001)

    parser = argparse.ArgumentParser(description="Jetracer Unified Runner (Normalized Control)")
    
    # 공통 제어 인자
    parser.add_argument("--speed5-throttle", type=float, default=None, help="속도 5.0일 때의 물리적 스로틀 목표값 (None인 경우 config 파일에서 로드)")
    parser.add_argument("--device", default=None, help="조이스틱 장치 경로 (예: /dev/input/event2)")
    parser.add_argument("--steer-scale", type=float, default=1.0, help="조이스틱 조향 배율")
    parser.add_argument("--log-calibration", action="store_true", help="속도 캘리브레이션용 데이터 로깅 활성화")
    parser.add_argument("--auto-calibrate", action="store_true", default=True, help="실시간 실제 속도 기반 스로틀 자동 보정 활성화 (기본값: True)")
    parser.add_argument("--no-auto-calibrate", action="store_false", dest="auto_calibrate", help="자동 보정 비활성화")
    parser.add_argument("--target-velocity", type=float, default=5.0, help="자동 보정 시 목표로 하는 실제 차량 속도 (m/s)")
    parser.add_argument("--auto-calibrate-packets", type=int, default=16, help="자동 보정 시 평균 속도를 계산할 패킷 개수 (기본값: 16, 약 0.5초)")
    parser.add_argument("--auto-calibrate-increment", type=float, default=default_inc, help=f"자동 보정 시 Stall Recovery 증가량 (기본값: {default_inc})")
    parser.add_argument("--auto-calibrate-decrement", type=float, default=default_dec, help=f"자동 보정 시 Speed Limit 감소량 (기본값: {default_dec})")
    parser.add_argument("--auto-calibrate-threshold", type=float, default=3.2, help="자동 보정이 트리거되는 평균 속도 임계값 (m/s)")
    parser.add_argument("--analyze", action="store_true", help="프로그램 종료 후 자동으로 캘리브레이션 데이터 분석 및 시각화 수행")
    parser.add_argument("--quiet-udp", action="store_true", help="UDP 모드 루틴 로그 숨기기 (에러/자동보정 요약은 표시)")
    parser.add_argument("--log-motor", action="store_true", help="모터 물리 신호 로그([motor]) 활성화")
    parser.add_argument("--telemetry-ip", default="192.168.0.100", help="텔레메트리 서버 IP")
    parser.add_argument("--telemetry-hz", type=float, default=30.0, help="텔레메트리 전송 주기 (Hz)")
    parser.add_argument("--car-number", type=int, default=None, help="차량 번호 (None이면 ID에서 유추)")
    parser.add_argument("--log-telemetry", action="store_true", help="텔레메트리 전송 로그 활성화")
    
    args = parser.parse_args()
    
    runner(args)
