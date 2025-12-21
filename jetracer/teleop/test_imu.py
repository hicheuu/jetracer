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

# [중요] 데이터 유실을 막았으므로 값이 더 커질 수 있습니다.
# 테스트 해보고 너무 크면 줄이세요.
SCALE_FACTOR = 3.3

# [수정] 14Hz(0.07초) 센서를 감안하여 넉넉하게 늘림
MAX_DT = 0.2  

def build_parser():
    p = argparse.ArgumentParser()
    p.add_argument("--server-ip", required=True)
    p.add_argument("--server-port", type=int, default=5560)
    p.add_argument("--hz", type=float, default=60.0)
    p.add_argument("--car-number", type=int, default=None)
    p.add_argument("--battery-shm-path", default="/dev/shm/jetracer_voltage")
    p.add_argument("--imu-port", default="/dev/ttyACM1")
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
    
    total_accumulated_yaw = 0.0
    serial_buffer = b""

    print(f"[시작] Gyro Z 적분 (손실 방지 모드). Scale Factor: {SCALE_FACTOR}")
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
                    
                    # 마지막 조각은 미완성이므로 버퍼에 남김
                    serial_buffer = parts[-1]
                    
                    # [핵심 수정] 버퍼에 있는 '모든' 완성된 패킷을 순서대로 처리
                    # parts[:-1]에는 완성된 문장들이 들어있음
                    valid_lines = parts[:-1]

                else:
                    continue

            except Exception:
                continue
            
            # 2. 버퍼에 있던 모든 패킷을 하나씩 다 적분함 (데이터 편식 금지)
            for raw_line in valid_lines:
                raw_line = raw_line.strip()
                if not (raw_line.startswith(b"#XYMU=") and raw_line.endswith(b"#")):
                    continue
                
                try:
                    line = raw_line.decode(errors="ignore").strip()
                    content = line.replace("#XYMU=", "").replace("#", "")
                    d = content.split(",")
                    
                    if len(d) < 10: continue
                    raw_gyro_z = float(d[9])
                except ValueError:
                    continue

                now = time.monotonic()
                if prev_t is None:
                    prev_t = now
                    continue

                dt = now - prev_t
                
                # dt가 너무 크면(0.2초 이상) 끊긴 걸로 간주하고 리셋
                # 하지만 정상적인 14Hz(0.07초) 데이터는 모두 통과시킴
                if dt <= 0.0 or dt > MAX_DT:
                    prev_t = now
                    continue

                # 3. 적분 수행
                step_yaw = (raw_gyro_z * dt) * SCALE_FACTOR
                acc_dyaw += step_yaw
                total_accumulated_yaw += step_yaw
                acc_dt += dt
                prev_t = now

            # 4. 전송 (루프 밖에서 누적된 값 전송)
            if acc_dt >= WINDOW_DT:
                seq += 1
                now_send = time.monotonic()
                if now_send >= next_allowed_send:
                    next_allowed_send = now_send + min_interval
                    
                    voltage = read_voltage(args.battery_shm_path) or 0.0
                    pkt = struct.pack(FMT_UPLINK, int(vehicle_id), float(voltage),
                                      float(acc_dyaw), float(acc_dt), int(seq))
                    sock.sendto(pkt, target)

                acc_dyaw = 0.0
                acc_dt = 0.0

    except KeyboardInterrupt:
        print("\n" + "="*40)
        total_deg = total_accumulated_yaw * 57.29578 
        print(f"🛑 테스트 종료")
        print(f"👉 총 회전 각도: {total_deg:.2f} 도")
        print("="*40)
             
    finally:
        ser.close()
        sock.close()

if __name__ == "__main__":
    main()