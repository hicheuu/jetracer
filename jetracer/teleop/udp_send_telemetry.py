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
PKT_SIZE = struct.calcsize(FMT_UPLINK)

TARGET_HZ = 30.0
WINDOW_DT = 1.0 / TARGET_HZ
YAW_SCALE = 3.0


MAX_YAW_RATE = 6.0
# [중요] dt 방어 기준을 더 엄격하게 낮춤 (0.1 -> 0.05)
# 0.05초(20Hz)보다 느리게 오면 데이터를 신뢰하지 않고 버림
MAX_DT = 0.05  

def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )

def quat_delta_yaw(q_prev, q_now):
    qd = quat_mul(quat_conj(q_prev), q_now)
    qw, qx, qy, qz = qd
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny, cosy)

def build_parser():
    p = argparse.ArgumentParser()
    p.add_argument("--server-ip", required=True)
    p.add_argument("--server-port", type=int, default=5560)
    p.add_argument("--hz", type=float, default=60.0)
    p.add_argument("--car-number", type=int, default=None)
    p.add_argument("--battery-shm-path", default="/dev/shm/jetracer_voltage")
    p.add_argument("--imu-port", default="/dev/ttyACM0")
    p.add_argument("--imu-baud", type=int, default=115200)
    p.add_argument("--poll-sleep", type=float, default=0.0001) # sleep 시간 극소화
    p.add_argument("--verbose", action="store_true")
    return p

def main():
    args = build_parser().parse_args()
    vehicle_id = infer_car_number(args.car_number)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.server_ip, args.server_port)

    # [핵심 변경 1] timeout=0 (완전 Non-blocking)
    ser = serial.Serial(args.imu_port, args.imu_baud, timeout=0)
    time.sleep(1)

    min_interval = 1.0 / max(1.0, float(args.hz))
    next_allowed_send = 0.0

    prev_q = None
    prev_t = None
    acc_dyaw = 0.0
    acc_dt = 0.0
    seq = 0

    # 데이터 처리를 위한 내부 버퍼
    serial_buffer = b""

    if args.verbose:
        print(f"[uplink] started. raw-parsing mode.")

    try:
        while True:
            # =======================================================
            # [핵심 변경 2] 수동 버퍼 파싱 (readline 사용 안 함)
            # =======================================================
            try:
                # 1. 현재 대기 중인 모든 바이트 읽기 (Non-blocking)
                waiting = ser.in_waiting
                if waiting > 0:
                    chunk = ser.read(waiting)
                    serial_buffer += chunk
                else:
                    # 데이터가 없으면 아주 짧게 쉬고 다시 확인
                    time.sleep(args.poll_sleep)
                    continue

                # 2. 줄바꿈(\n) 기준으로 패킷 분리
                if b'\n' in serial_buffer:
                    # 마지막 줄바꿈 기준으로 split
                    parts = serial_buffer.split(b'\n')
                    
                    # parts[-1]은 줄바꿈 뒤에 남은 '미완성 데이터'이므로 버퍼에 남김
                    serial_buffer = parts[-1]
                    
                    # parts[:-1]은 완성된 라인들임. 그 중 '가장 마지막' 유효 패킷만 사용
                    valid_line = None
                    
                    # 뒤에서부터 탐색 (최신 데이터 우선)
                    for raw_line in reversed(parts[:-1]):
                        # 간단한 유효성 검사 (앞뒤 공백 제거 후 체크)
                        r = raw_line.strip()
                        if r.startswith(b"#XYMU=") and r.endswith(b"#"):
                            valid_line = r
                            break # 최신을 찾았으니 루프 종료
                    
                    if valid_line:
                        line = valid_line.decode(errors="ignore").strip()
                    else:
                        continue # 이번 덩어리엔 유효 패킷이 없었음
                else:
                    # 줄바꿈이 아직 안 들어옴 -> 데이터 더 쌓일 때까지 대기
                    continue

            except OSError:
                time.sleep(0.1)
                continue
            except Exception as e:
                if args.verbose: print(f"Parse Error: {e}")
                continue
            
            # =======================================================
            # 데이터 파싱 및 로직 (이전과 동일하지만 dt 로직 강화)
            # =======================================================
            d = line[6:-1].split(",")
            if len(d) < 7: continue

            try:
                qw, qx, qy, qz = map(float, d[3:7])
            except ValueError: continue

            q_now = (qw, qx, qy, qz)
            now = time.monotonic()

            if prev_q is None:
                prev_q = q_now
                prev_t = now
                continue

            dt = now - prev_t

            # [핵심 변경 3] dt 필터링 강화
            # 스파이크가 튀면(0.05초 이상 지연) 해당 프레임의 '변화량'을 0으로 처리하거나 리셋
            # 여기서는 리셋 전략 사용
            if dt <= 0.0 or dt > MAX_DT:
                # dt가 튀었다는 것은 그 사이 데이터가 소실되었거나 지연되었다는 뜻.
                # 이 구간의 dyaw를 적분하면 값이 튀므로, 적분하지 않고 기준점(prev_t)만 갱신
                prev_q = q_now
                prev_t = now
                # (옵션) 디버깅용
                # if args.verbose: print(f"[Skip] dt spike: {dt:.4f}")
                continue

            dyaw = quat_delta_yaw(prev_q, q_now) * YAW_SCALE

            # 물리 한계 방어
            if abs(dyaw) > MAX_YAW_RATE * dt:
                # prev_q 유지 (이번 데이터 노이즈로 간주)
                continue

            prev_q = q_now
            prev_t = now

            acc_dyaw += dyaw
            acc_dt += dt

            if acc_dt >= WINDOW_DT:
                # ... (전송 로직 동일) ...
                seq += 1
                
                # 전송 주기 제어
                now_send = time.monotonic()
                if now_send >= next_allowed_send:
                    next_allowed_send = now_send + min_interval
                    
                    voltage = read_voltage(args.battery_shm_path) or 0.0
                    
                    pkt = struct.pack(FMT_UPLINK, int(vehicle_id), float(voltage),
                                      float(acc_dyaw), float(acc_dt), int(seq))
                    sock.sendto(pkt, target)
                    
                    if args.verbose:
                        print(f"UDP send: dψ={acc_dyaw:+.4f} dt={acc_dt:.4f}")

                # 누적값 리셋
                acc_dyaw = 0.0
                acc_dt = 0.0

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        sock.close()

if __name__ == "__main__":
    main()