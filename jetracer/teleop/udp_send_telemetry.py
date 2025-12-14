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

# =========================
# Binary packet definition
# =========================
FMT_UPLINK = "!ifffI"
PKT_SIZE = struct.calcsize(FMT_UPLINK)  # 20 bytes

# =========================
# IMU Quaternion utils
# =========================
TARGET_HZ = 30.0
WINDOW_DT = 1.0 / TARGET_HZ

# 방어 파라미터
MAX_YAW_RATE = 6.0     # rad/s
MAX_DT = 0.10          # sec


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
    # q_delta = q_prev^-1 ⊗ q_now
    qd = quat_mul(quat_conj(q_prev), q_now)
    qw, qx, qy, qz = qd

    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny, cosy)


def build_parser():
    p = argparse.ArgumentParser(description="UDP telemetry sender (binary uplink)")
    p.add_argument("--server-ip", required=True, help="server IP")
    p.add_argument("--server-port", type=int, default=5560, help="server port")

    # NOTE: --hz는 "최대 전송률 상한"으로만 사용 (기본 60Hz)
    p.add_argument("--hz", type=float, default=60.0, help="max send rate cap (Hz)")

    p.add_argument("--car-number", type=int, default=None)
    p.add_argument("--battery-shm-path", default="/dev/shm/jetracer_voltage")
    
    # IMU 시리얼 포트 설정
    p.add_argument("--imu-port", default="/dev/ttyACM0", help="IMU serial port")
    p.add_argument("--imu-baud", type=int, default=115200, help="IMU serial baud rate")

    # CPU 과점유 방지용 폴링 슬립 (초)
    p.add_argument("--poll-sleep", type=float, default=0.001, help="poll sleep seconds (default: 0.001)")

    p.add_argument("--verbose", action="store_true")
    return p


def main():
    args = build_parser().parse_args()

    vehicle_id = infer_car_number(args.car_number)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.server_ip, args.server_port)

    # IMU 시리얼 포트 열기
    # timeout을 아주 짧게 주거나 0으로 주어 non-blocking에 가깝게 동작하도록 함
    ser = serial.Serial(args.imu_port, args.imu_baud, timeout=0.01)
    time.sleep(1)

    # 최대 전송률 상한(너무 빨리 도는 것 방지)
    min_interval = 1.0 / max(1.0, float(args.hz))
    next_allowed_send = 0.0

    # IMU 상태 변수
    prev_q = None
    prev_t = None
    acc_dyaw = 0.0
    acc_dt = 0.0
    seq = 0

    if args.verbose:
        print(
            f"[uplink] target={target} "
            f"fmt={FMT_UPLINK} size={PKT_SIZE}B vehicle_id={vehicle_id} "
            f"max_hz={args.hz} poll_sleep={args.poll_sleep} "
            f"imu_port={args.imu_port} imu_baud={args.imu_baud}"
        )

    try:
        while True:
            # =================================================================
            # [수정된 부분 Start] Serial Buffer Flushing Strategy
            # =================================================================
            # 버퍼에 쌓인 데이터가 있는지 확인
            if ser.in_waiting > 0:
                last_valid_line = None
                
                # 쌓여있는 모든 라인을 읽어내고, 가장 마지막(최신) 유효 라인만 건짐
                while ser.in_waiting > 0:
                    try:
                        raw = ser.readline()
                        # 최소한의 바이트 레벨 검증 (디코딩 비용 절약)
                        # #XYMU=로 시작하는지 체크 (바이트열 기준)
                        if raw.startswith(b"#XYMU=") and (b"#" in raw): 
                            last_valid_line = raw
                    except Exception:
                        pass
                
                if last_valid_line is None:
                    time.sleep(args.poll_sleep)
                    continue
                
                # 최신 라인 디코딩
                line = last_valid_line.decode(errors="ignore").strip()
            
            else:
                # 데이터가 없으면 잠시 대기
                time.sleep(args.poll_sleep)
                continue
            # =================================================================
            # [수정된 부분 End]
            # =================================================================

            # 엄격한 포맷 검사
            if not (line.startswith("#XYMU=") and line.endswith("#")):
                continue

            d = line[6:-1].split(",")
            if len(d) < 7:
                continue

            try:
                qw, qx, qy, qz = map(float, d[3:7])
            except ValueError:
                continue

            q_now = (qw, qx, qy, qz)
            now = time.monotonic()

            # 초기화
            if prev_q is None:
                prev_q = q_now
                prev_t = now
                continue

            dt = now - prev_t

            # dt 방어 + 재동기화
            if dt <= 0.0 or dt > MAX_DT:
                # 스파이크가 발생했거나 너무 오래된 경우 리셋
                acc_dyaw = 0.0
                acc_dt = 0.0
                prev_q = q_now
                prev_t = now
                continue

            # quaternion delta 기반 yaw 계산
            dyaw = quat_delta_yaw(prev_q, q_now)

            # 물리 한계 방어 (노이즈 필터링)
            if abs(dyaw) > MAX_YAW_RATE * dt:
                # 물리적으로 불가능한 회전 -> 튀는 값으로 간주하고 무시
                # prev_q는 업데이트하지 않음 (튀는 값은 버림)
                continue

            # 정상 프레임 처리
            prev_q = q_now
            prev_t = now

            acc_dyaw += dyaw
            acc_dt += dt

            # 30Hz 윈도우 체크 및 전송
            if acc_dt >= WINDOW_DT:
                seq += 1
                heading_diff = acc_dyaw
                heading_dt = acc_dt

                # 전송률 상한 체크
                now_send = time.monotonic()
                if now_send < next_allowed_send:
                    # 너무 빠르면 이번 전송은 스킵하되, 누적값은 리셋해야 함
                    # (혹은 다음 주기에 합쳐서 보낼 수도 있지만, 여기선 리셋 정책 유지)
                    pass 
                else:
                    next_allowed_send = now_send + min_interval

                    # voltage 읽기
                    voltage = read_voltage(args.battery_shm_path)
                    if voltage is None:
                        voltage = 0.0

                    # 패킷 전송
                    pkt = struct.pack(
                        FMT_UPLINK,
                        int(vehicle_id),
                        float(voltage),
                        float(heading_diff),
                        float(heading_dt),
                        int(seq),
                    )
                    sock.sendto(pkt, target)

                    if args.verbose:
                        print(
                            f"[uplink] vid={vehicle_id} "
                            f"V={voltage:.2f} "
                            f"dψ={float(heading_diff):+.6f} "
                            f"dt={float(heading_dt):.4f} "
                            f"seq={int(seq)}"
                        )
                
                # 누적값 리셋 (전송 여부와 상관없이 윈도우가 찼으면 리셋)
                acc_dyaw = 0.0
                acc_dt = 0.0

    except KeyboardInterrupt:
        if args.verbose:
            print("\n[uplink] interrupted")
    finally:
        ser.close()
        sock.close()


if __name__ == "__main__":
    main()