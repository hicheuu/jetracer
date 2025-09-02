#!/usr/bin/env python3
import socket
import struct
import time
import math
import argparse

from jetracer.nvidia_racecar import NvidiaRacecar

# 송신 포맷: "!ffI" = big-endian, float(throttle 0..1), float(steering_deg -deg_max..+deg_max), uint32(seq)
FMT = "!ffI"
PACKET_SIZE = struct.calcsize(FMT)

def clamp(x, lo=-1.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x

def is_finite(x):
    return (x is not None) and (not math.isnan(x)) and (not math.isinf(x))

def main():
    parser = argparse.ArgumentParser(description="UDP receiver -> Jetracer driver (slow & high-resolution control)")
    parser.add_argument("--bind-ip", default="0.0.0.0", help="수신 바인드 IP (기본: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=5555, help="수신 포트 (기본: 5555)")
    parser.add_argument("--watchdog", type=float, default=1.0, help="워치독 타임아웃 초 (기본: 1.0s)")
    parser.add_argument("--deg-max", type=float, default=30.0, help="송신 각도의 풀스케일 (기본 ±30 deg)")
    parser.add_argument("--center-norm", type=float, default=0.0,
                        help="실제 직진이 되는 정규화 스티어 값(예: -0.2). 시작/워치독 정렬 및 정규화 바이어스에 사용")

    # (1) 저속 고분해능: 스케일 + 엑스포(곡선) 맵핑
    parser.add_argument("--thr-scale", type=float, default=0.30,
                        help="최고속도 스케일(0~1). 예: 0.30 → 최고속도 30%%로 제한(저속 분해능↑)")
    parser.add_argument("--expo", type=float, default=0.60,
                        help="저속 강조 곡률(0<expo<=1, 작을수록 저속에서 더 민감). 권장 0.5~0.8")

    # (2) 킥 & 홀드: 단 1회만 킥 (정지→출발 시에만)
    parser.add_argument("--start-thresh", type=float, default=0.15,
                        help="출발 임계 스로틀(이하에선 본래 안 굴러감). 네 차량 기준 기본 0.15")
    parser.add_argument("--kick", type=float, default=0.155,
                        help="출발 킥 스로틀(임계점 약간 위). 기본 0.18")
    parser.add_argument("--kick-ms", type=int, default=100,
                        help="킥 지속시간(ms). 기본 120ms")

    # (3) 펄스-밀도(디더링)
    parser.add_argument("--pdm-hz", type=float, default=25.0,
                        help="디더링 주기(Hz). 15~40 권장(높을수록 덜 덜컹)")
    parser.add_argument("--deadzone", type=float, default=0.02,
                        help="노이즈 제거용 미소 데드존. 이하면 0으로 간주")

    # 루프/수신 타임아웃(디더링을 위해 빠르게 도는 게 유리)
    parser.add_argument("--recv-timeout", type=float, default=0.02,
                        help="recv 타임아웃(초). 디더링 위해 10~30ms 권장")
    parser.add_argument("--verbose", action="store_true", help="수신/맵핑/디더링 로그 출력")
    args = parser.parse_args()

    car = NvidiaRacecar()

    # 시작: 실제 센터 정렬 + 정지
    car.steering = args.center_norm
    car.throttle = 0.0

    # 소켓
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind_ip, args.port))
    # argparse 속성명에서 하이픈은 언더스코어로 바뀜: recv_timeout
    sock.settimeout(args.recv_timeout)

    last_rx_ts = time.time()
    last_seq = None

    # 디더링 상태
    pdm_dt = 1.0 / max(1e-3, args.pdm_hz)
    last_pdm = time.time()
    accum = 0.0  # 브레젠험 누적자

    # 목표 스로틀(수신 없을 때도 유지)
    target_throttle = 0.0

    # 킥 1회 제어용 상태 플래그:
    # - stopped: 지금 정지 상태인지(=최근 출력이 사실상 0 근처였는지)
    # - kick_done: 현재 "저속 유지 구간"에서 킥을 이미 수행했는지
    stopped = True
    kick_done = False

    print(f"[UDP] listening on {args.bind_ip}:{args.port}  fmt={FMT} ({PACKET_SIZE} bytes)")
    print(f"[MAP] steer_norm = (deg / {args.deg_max}) + center_norm  (center_norm={args.center_norm:+.2f})")
    print(f"[INFO] slow-res cfg: thr_scale={args.thr_scale:.2f}, expo={args.expo:.2f}, "
          f"start_thresh={args.start_thresh:.2f}, kick={args.kick:.2f}/{args.kick_ms}ms, "
          f"pdm_hz={args.pdm_hz:.1f}, deadzone={args.deadzone:.3f}")
    print("[TIP ] ESC Training Mode(50%% 제한)를 켜면 저속 안정성이 더 좋아집니다. (EZ-Set 빨강 3번 깜박)")

    try:
        while True:
            # === 수신 ===
            data = None
            try:
                data, addr = sock.recvfrom(64)
            except socket.timeout:
                pass

            if data and len(data) >= PACKET_SIZE:
                throttle_in, steering_deg, seq = struct.unpack(FMT, data[:PACKET_SIZE])

                # NaN/Inf 방어
                if is_finite(throttle_in) and is_finite(steering_deg):
                    last_seq = seq if last_seq is None else seq
                    last_rx_ts = time.time()

                    # 조향 맵핑
                    raw_norm = steering_deg / float(args.deg_max)
                    steer_norm = clamp(raw_norm + args.center_norm, -1.0, 1.0)
                    car.steering = steer_norm

                    # 스로틀 맵핑(엑스포 + 스케일)
                    thr_in = clamp(throttle_in, 0.0, 1.0)
                    expo = max(1e-3, min(1.0, args.expo))
                    thr_shaped = thr_in ** expo          # 0~1
                    target_throttle = thr_shaped * max(0.0, min(1.0, args.thr_scale))  # 0 ~ thr_scale

                    if args.verbose:
                        print(f"seq={seq} in_thr={thr_in:+.3f} shaped={thr_shaped:+.3f} "
                              f"target={target_throttle:+.3f} deg={steering_deg:+.1f} "
                              f"steer_norm={steer_norm:+.3f}")

            # === 워치독: 미수신 시 안전 정지 & 상태 초기화 ===
            now = time.time()
            if (now - last_rx_ts) > args.watchdog:
                # 센터 + 정지
                if (car.steering != args.center_norm) or (target_throttle != 0.0):
                    car.steering = args.center_norm
                    target_throttle = 0.0
                    if args.verbose:
                        print(f"[watchdog] no packets > {args.watchdog:.2f}s → center & stop")
                last_rx_ts = now  # 반복 로그 억제
                # 상태 초기화(다음에 다시 들어올 때 킥 1회 허용)
                stopped = True
                kick_done = False

            # === 출력 단계: 킥(1회) + PDM(디더링) ===
            DEAD = max(0.0, args.deadzone)
            ST = max(0.0, args.start_thresh)  # 출발 임계

            # 타깃 정리(데드존 억제)
            if 0.0 < target_throttle < DEAD:
                target = 0.0
            else:
                target = target_throttle

            # 현재 상태 추정(아주 단순: 출력이 0 이하면 정지로 간주)
            # 실제로는 엔코더/IMU가 있다면 그걸로 판단하는 게 정확함
            if target <= 0.0:
                stopped = True
                kick_done = False  # 다음에 다시 출발할 때 킥 허용

            # 디더링 타이밍에 맞춰 실행
            if now - last_pdm >= pdm_dt:
                last_pdm = now

                if 0.0 < target < ST:
                    # 저속 유지 구간: 킥은 '정지→출발' 첫 1회만
                    if stopped and not kick_done:
                        # 킥 1회
                        car.throttle = clamp(args.kick, 0.0, 1.0)
                        time.sleep(args.kick_ms / 1000.0)
                        kick_done = True
                        stopped = False  # 이제 움직이는 상태로 간주

                    # 킥 이후에는 PDM으로 평균값(target) 유지 (킥 반복 없음)
                    accum += (target / ST)
                    if accum >= 1.0:
                        car.throttle = ST   # ON
                        accum -= 1.0
                        if args.verbose:
                            print(f"[pdm] ON  (avg={target:.3f}, ST={ST:.3f}, acc={accum:.3f})")
                    else:
                        car.throttle = 0.0  # OFF
                        if args.verbose:
                            print(f"[pdm] OFF (avg={target:.3f}, ST={ST:.3f}, acc={accum:.3f})")
                else:
                    # 임계 이상 또는 완전 정지 → 원하는 값 그대로 출력
                    car.throttle = clamp(target, 0.0, 1.0)
                    # 임계 이상이면 움직이는 중으로 간주
                    if target >= ST:
                        stopped = False
                        kick_done = True  # 이 구간에서는 킥 필요 없음

    except KeyboardInterrupt:
        pass
    finally:
        try:
            car.throttle = 0.0
            car.steering = args.center_norm
        except Exception:
            pass
        sock.close()
        print("exiting... motors stopped (centered).")

if __name__ == "__main__":
    main()

