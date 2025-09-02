#!/usr/bin/env python3
import re, sys, math, json, socket, time
from collections import deque
from datetime import datetime
import serial

# ==========================
# 고정 실행 설정
# ==========================
CONFIG = {
    # 시리얼 포트 (by-id 사용 권장)
    "serial_port": "/dev/serial/by-id/usb-SparkFun_SFE_9DOF-D21_F22F6F2A50533357322E3120FF090637-if00",
    "baudrate": 115200,

    # UDP 전송 대상
    "dest_ip": "172.20.10.2",   # ← 여기 원하는 IP로 수정
    "dest_port": 7777,

    # 동작 파라미터
    "lp_window": 10,   # yaw moving average 샘플 수
    "rate_hz": 30.0,   # UDP 전송 최대 주기 (Hz)
    "print_every": 5   # 몇 샘플마다 터미널 출력할지
}
# ==========================

LINE_RE = re.compile(r'#\s*XYMU\s*=\s*([^\#]+)\s*#')

def parse_xymu(line: str):
    m = LINE_RE.search(line)
    if not m:
        return None
    try:
        vals = [float(x) for x in m.group(1).split(',')]
        if len(vals) < 9:
            return None
        ax, ay, az, gx, gy, gz, mx, my, mz = vals[:9]
        return ax, ay, az, gx, gy, gz, mx, my, mz
    except:
        return None

def yaw_from_acc_mag(ax, ay, az, mx, my, mz):
    roll  = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    mx_c = mx*math.cos(pitch) + mz*math.sin(pitch)
    my_c = mx*math.sin(roll)*math.sin(pitch) + my*math.cos(roll) - mz*math.sin(roll)*math.cos(pitch)
    yaw = math.atan2(-my_c, mx_c)
    yaw_deg = math.degrees(yaw)
    if yaw_deg < 0:
        yaw_deg += 360.0
    return yaw_deg

def main():
    port = CONFIG["serial_port"]
    baud = CONFIG["baudrate"]
    dest = (CONFIG["dest_ip"], CONFIG["dest_port"])
    lp = CONFIG["lp_window"]
    min_dt = 1.0 / max(1e-6, CONFIG["rate_hz"])
    print_every = CONFIG["print_every"]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    yaw_buf = deque(maxlen=max(1, lp))
    last_send = 0.0
    sample_idx = 0

    with serial.Serial(port, baud, timeout=1) as ser:
        ser.reset_input_buffer()
        print(f'Listening {port} @ {baud} → UDP {dest}  (Ctrl+C to quit)')
        while True:
            line = ser.readline().decode('utf-8', errors='ignore')
            if not line:
                continue
            parsed = parse_xymu(line)
            if not parsed:
                continue
            ax, ay, az, gx, gy, gz, mx, my, mz = parsed
            yaw_deg = yaw_from_acc_mag(ax, ay, az, mx, my, mz)
            yaw_buf.append(yaw_deg)
            yaw_smooth = sum(yaw_buf) / len(yaw_buf)

            heading_deg = yaw_smooth  # 여기서는 heading = yaw 로 정의

            now = time.time()
            if now - last_send >= min_dt:
                payload = {
                    "ts": datetime.utcnow().isoformat() + "Z",
                    "yaw_deg": round(yaw_deg, 3),
                    "heading_deg": round(heading_deg, 3)
                }
                sock.sendto(json.dumps(payload).encode('utf-8'), dest)
                last_send = now

                sample_idx += 1
                if sample_idx % max(1, print_every) == 0:
                    print(f"send → {dest}  yaw={yaw_deg:7.2f}°, heading={heading_deg:7.2f}°  (lp={len(yaw_buf)})")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n종료")
