import socket
import struct
import threading
import time

# jetracer 패키지에서 NvidiaRacecar 가져오기
from jetracer.nvidia_racecar import NvidiaRacecar

IP   = "10.222.125.128"   # 이 보드의 Wi-Fi IP
PORT = 5555
FMT  = "!ffI"             # throttle(float), steering(float), seq(uint32)
PACKET_SIZE = struct.calcsize(FMT)

# 안전 범위 클램프
def clamp(x, lo=-1.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x

class UdpReceiver:
    def __init__(self, ip, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((ip, port))
        self.sock.settimeout(1.0)
        self.last_rx_ts = time.time()
        self.last_seq = None
        self.running = True

    def recv_once(self):
        try:
            data, addr = self.sock.recvfrom(64)
        except socket.timeout:
            return None
        if len(data) < PACKET_SIZE:
            return None
        throttle, steering, seq = struct.unpack(FMT, data[:PACKET_SIZE])
        self.last_rx_ts = time.time()
        # seq 관리(중복 drop은 선택)
        if self.last_seq is not None and seq == self.last_seq:
            return None
        self.last_seq = seq
        return throttle, steering, seq

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass

def main():
    car = NvidiaRacecar()
    rx  = UdpReceiver(IP, PORT)

    # 워치독: 일정 시간 이상 패킷 없으면 정지
    WATCHDOG_SEC = 1.0

    print(f"Listening on {IP}:{PORT}  (fmt={FMT}, {PACKET_SIZE} bytes)")
    try:
        while True:
            msg = rx.recv_once()
            now = time.time()

            if msg is not None:
                thr, st, seq = msg
                # 안전 범위로 클램프
                thr = clamp(thr)
                st  = clamp(st)
                # 적용 (NvidiaRacecar 내부에서 소프트스타트/게인 처리)
                car.throttle = thr
                car.steering = st
                # 디버그 출력(필요 없으면 주석)
                # print(f"thr={thr:+.3f}  steer={st:+.3f}  seq={seq}")

            # 워치독: 패킷 끊기면 즉시 정지
            if (now - rx.last_rx_ts) > WATCHDOG_SEC:
                if car.throttle != 0.0:
                    car.throttle = 0.0
                # steering은 그대로 두거나 0.0으로: 여기선 안전하게 0.0
                car.steering = 0.0
                rx.last_rx_ts = now  # 과도한 반복 정지 방지

    except KeyboardInterrupt:
        pass
    finally:
        try:
            car.throttle = 0.0
            car.steering = 0.0
        except Exception:
            pass
        rx.close()

if __name__ == "__main__":
    main()

