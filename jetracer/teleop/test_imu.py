#!/usr/bin/env python3
from __future__ import annotations

import argparse
import socket
import struct
import time
import serial
import math

from jetracer.teleop.telemetry_common import (
    infer_car_number,
    read_voltage,
)

FMT_UPLINK = "!ifffI"

TARGET_HZ = 30.0
WINDOW_DT = 1.0 / TARGET_HZ

# [보정 계수]
# 90도 돌렸는데 결과가 45도면 -> 60.0으로 올리세요
# 90도 돌렸는데 결과가 180도면 -> 15.0으로 줄이세요
SCALE_FACTOR = 30.0 

MAX_DT = 0.05  

def build_parser():
    p = argparse.ArgumentParser()
    p.add_argument("--server-ip", required=True)
    p.add_argument("--server-port", type=int, default=5560)
    p.add_argument("--hz", type=float, default=60.0)
    p.add_argument("--car-number", type=int, default=None)
    p.add_argument("--battery-shm-path", default="/dev/shm/jetracer_voltage")
    p.add_argument("--imu-port", default="/dev/ttyUSB0")
    p.add_argument("--imu-baud", type=int, default=115200)
    p.add_argument("--poll-sleep", type=float, default=0.0001)
    p.add_argument("--verbose", action="store_true")
    return p

def main():
    args = build_parser().parse_args()
    vehicle_id = infer_car_number(args.car_number)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.server_ip, args.server_port)

    try:
        ser = serial.Serial(args.imu_port, args.imu_baud, timeout=0)
    except Exception as e:
        print(f"[ERROR] IMU 연결 실패: {e}")
        return

    time.sleep(1)

    min_interval = 1.0 / max(1.0, float(args.hz))
    next_allowed_send = 0.0

    prev_t = None
    acc_dyaw = 0.0
    acc_dt = 0.0
    seq = 0
    
    # [추가] 전체 누적 각도 (종료 시 확인용)
    total_accumulated_yaw = 0.0

    serial_buffer = b""

    print(f"[시작] Gyro Z 적분 모드. Scale Factor: {SCALE_FACTOR}")
    print("테스트 방법: 로봇을 90도 돌리고 Ctrl+C를 누르세요.")

    try:
        while True:
            # 1. 시리얼 읽기
            try:
                waiting = ser.in_waiting
                if waiting > 0:
                    chunk = ser.read(waiting)
                    serial_buffer += chunk
                else:
                    time.sleep(args.poll_sleep)
                    continue

                if b'\n' in serial_buffer:
                    parts = serial_buffer.split(b'\n')
                    serial_buffer = parts[-1]
                    
                    valid_line = None
                    for raw_line in reversed(parts[:-1]):
                        r = raw_line.strip()
                        if r.startswith(b"#XYMU=") and r.endswith(b"#"):
                            valid_line = r
                            break 
                    
                    if valid_line:
                        line = valid_line.decode(errors="ignore").strip()
                    else:
                        continue
                else:
                    continue

            except Exception:
                continue
            
            # 2. Gyro Z 파싱
            content = line.replace("#XYMU=", "").replace("#", "")
            d = content.split(",")
            if len(d) < 10: continue

            try:
                raw_gyro_z = float(d[9]) # 자이로 Z값
            except ValueError: continue

            now = time.monotonic()
            if prev_t is None:
                prev_t = now
                continue

            dt = now - prev_t
            if dt <= 0.0 or dt > MAX_DT:
                prev_t = now
                continue

            # 3. 각도 적분 (속도 * 시간 * 보정계수)
            step_yaw = (raw_gyro_z * dt) * SCALE_FACTOR
            
            acc_dyaw += step_yaw
            total_accumulated_yaw += step_yaw # 전체 누적값에 더하기
            acc_dt += dt
            prev_t = now

            # 4. 전송
            if acc_dt >= WINDOW_DT:
                seq += 1
                now_send = time.monotonic()
                if now_send >= next_allowed_send:
                    next_allowed_send = now_send + min_interval
                    
                    voltage = read_voltage(args.battery_shm_path) or 0.0
                    pkt = struct.pack(FMT_UPLINK, int(vehicle_id), float(voltage),
                                      float(acc_dyaw), float(acc_dt), int(seq))
                    sock.sendto(pkt, target)
                    
                    # (옵션) 실시간으로 보고 싶으면 주석 해제
                    # if args.verbose:
                    #     print(f"UDP 전송: 변화량 {acc_dyaw*57.2958:.2f}도")

                acc_dyaw = 0.0
                acc_dt = 0.0

    except KeyboardInterrupt:
        print("\n" + "="*40)
        # 라디안 -> 도 변환 (rad * 180 / pi)
        total_deg = total_accumulated_yaw * 57.29578 
        print(f"🛑 테스트 종료")
        print(f"👉 총 회전 각도: {total_deg:.2f} 도")
        print("="*40)
        
        # 팁 출력
        if abs(total_deg) < 5.0:
            print("💡 팁: 각도가 너무 작습니다. Scale Factor를 크게 키우세요.")
        elif abs(total_deg) > 360.0:
             print("💡 팁: 각도가 너무 큽니다. Scale Factor를 줄이세요.")
             
    finally:
        ser.close()
        sock.close()

if __name__ == "__main__":
    main()