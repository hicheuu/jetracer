

#!/usr/bin/env python3
"""
Simple UDP Receiver -> UDS Sender
- Format: !fiI (float steer, int speed, uint seq)
- speed=5 -> throttle=0.339 유지
"""
import socket
import struct
import time
import json

# =========================
# UDS 설정
# =========================
SOCK_PATH = "/tmp/jetracer_ctrl.sock"
udsock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

# =========================
# UDP 설정
# =========================
UDP_IP = "0.0.0.0"
UDP_PORT = 5555

# =========================
# 제어 파라미터
# =========================
STEER_GAIN = 1.0
# STEER_OFFSET은 제거 - nvidia_racecar.py에서 config 파일의 offset을 적용함

# =========================
# Throttle 매핑 설정 (직접 입력)
# =========================
# 아래 두 값을 직접 수정하세요:
SPEED_1_THROTTLE = 0.331  # speed=1일 때 throttle 값
SPEED_5_THROTTLE = 0.341  # speed=5일 때 throttle 값
# 이 두 값으로 선형 보간하여 다른 speed 값의 throttle을 계산합니다

MAX_THROTTLE = 0.36
ESC_NEUTRAL = 0.12

WATCHDOG_TIMEOUT = 1.0


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def speed_to_throttle(speed):
    """speed 값을 throttle로 변환 (선형 보간 사용)
    
    Args:
        speed: speed 값 (0 이상)
        
    Returns:
        throttle 값
    """
    if speed == 0:
        return ESC_NEUTRAL
    
    # SPEED_1_THROTTLE과 SPEED_5_THROTTLE로 선형 보간
    # 두 점 (1, SPEED_1_THROTTLE)과 (5, SPEED_5_THROTTLE)을 지나는 직선
    slope = (SPEED_5_THROTTLE - SPEED_1_THROTTLE) / (5.0 - 1.0)
    intercept = SPEED_1_THROTTLE - slope * 1.0
    
    throttle = slope * float(speed) + intercept
    return clamp(throttle, ESC_NEUTRAL, ESC_NEUTRAL + MAX_THROTTLE)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    print(f"[UDP] Listening on {UDP_IP}:{UDP_PORT}")
    print(f"[UDP] Format = !fiI | speed=1 -> thr={SPEED_1_THROTTLE:.3f}, speed=5 -> thr={SPEED_5_THROTTLE:.3f}")

    last_rx_time = time.time()

    try:
        while True:
            try:
                data, _ = sock.recvfrom(64)

                if len(data) >= 12:
                    raw_steer, raw_speed, seq = struct.unpack("!fiI", data[:12])

                    # ===== steering =====
                    # offset은 nvidia_racecar.py에서 config 파일의 값을 사용하여 적용됨
                    steer_val = raw_steer * STEER_GAIN
                    steer_cmd = clamp(steer_val, -1.0, 1.0)

                    # ===== throttle =====
                    throttle_cmd = speed_to_throttle(raw_speed)

                    # ===== UDS 송신 =====
                    udsock.sendto(
                        json.dumps({
                            "src": "udp",
                            "steer": steer_cmd,
                            "throttle": throttle_cmd
                        }).encode(),
                        SOCK_PATH
                    )

                    last_rx_time = time.time()

                    if seq % 20 == 0:
                        print(
                            f"[UDP→MUX] seq={seq:<5} "
                            f"speed={raw_speed} -> thr={throttle_cmd:.3f} "
                            f"steer={steer_cmd:+.3f}"
                        )

            except BlockingIOError:
                pass
            except struct.error:
                pass

            # ===== watchdog =====
            if time.time() - last_rx_time > WATCHDOG_TIMEOUT:
                udsock.sendto(
                    json.dumps({
                        "src": "udp",
                        "steer": 0.0,
                        "throttle": ESC_NEUTRAL
                    }).encode(),
                    SOCK_PATH
                )
                last_rx_time = time.time()
                print("[UDP] watchdog → neutral")

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[UDP] stopping")
    finally:
        sock.close()


if __name__ == "__main__":
    main()










