#!/usr/bin/env python3
import re, sys, math, json, socket, time
from collections import deque
from datetime import datetime
import serial

CONFIG = {
    # ── 시리얼/UDP 설정 ─────────────────────────────────────────────────────────
    "serial_port": "/dev/ttyACM0",   # 필요시 /dev/serial/by-id/... 로 교체
    "baudrate": 115200,

    "dest_ip": "172.20.10.2",
    "dest_port": 7777,

    "lp_window": 10,      # yaw 이동평균 샘플 수
    "rate_hz": 30.0,      # 최대 전송 주기(Hz)
    "print_every": 5,     # 몇 샘플마다 로그 출력할지

    # ── 자력계 축 매핑/부호 (XYMU: v[6],v[7],v[8] = mx,my,mz 가정) ────────────
    "m_idx":  (6, 7, 8),           # (mx_i, my_i, mz_i)  필요시 (7,6,8) 등으로 변경
    "m_sign": (+1, +1, +1),        # 각 축 부호(±1)

    # ── 캘리브레이션 결과 반영 (하드아이언/소프트아이언) ──────────────────────
    # 90초 수집 결과:
    # m_bias  = (0.055000, -0.600000, 0.000000)
    # m_scale = (38.731922, 0.680835, 0.664277)
    "m_bias":  (0.055000, -0.600000, 0.000000),
    "m_scale": (38.731922, 0.680835, 0.664277),

    # ── yaw 최종 보정(규약/오프셋/자기편차) ─────────────────────────────────────
    "use_nav_atan2": True,   # True: atan2(-my_c, mx_c)  (북=0°, 동=+90°)
    "yaw_sign": +1,          # 필요 시 -1
    "yaw_offset_deg": 0.0,   # 장착 오프셋(±90/±180 등)
    "declination_deg": 0.0   # 자북→진북 보정(서울 대략 +7~8°; 원하면 값 넣기)
}

LINE_RE = re.compile(r'#\s*XYMU\s*=\s*([^\#]+)\s*#')

def parse_vals(line: str):
    m = LINE_RE.search(line)
    if not m:
        return None
    try:
        vals = [float(x) for x in m.group(1).split(',')]
        return vals if len(vals) >= 9 else None
    except:
        return None

def wrap_360(deg: float) -> float:
    deg = deg % 360.0
    return deg + 360.0 if deg < 0 else deg

def compute_yaw(ax, ay, az, mx, my, mz, use_nav=True, yaw_sign=+1, yaw_off=0.0, decl=0.0):
    # roll, pitch (라디안)
    roll  = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))

    # tilt compensation
    mx_c = mx*math.cos(pitch) + mz*math.sin(pitch)
    my_c = mx*math.sin(roll)*math.sin(pitch) + my*math.cos(roll) - mz*math.sin(roll)*math.cos(pitch)

    # yaw (라디안)
    yaw  = math.atan2(-my_c, mx_c) if use_nav else math.atan2(my_c, mx_c)

    # deg 변환 + 보정
    yaw_deg = math.degrees(yaw)
    yaw_deg = yaw_deg * yaw_sign + yaw_off + decl
    return wrap_360(yaw_deg)

def main():
    port = CONFIG["serial_port"]; baud = CONFIG["baudrate"]
    dest = (CONFIG["dest_ip"], CONFIG["dest_port"])
    lp = CONFIG["lp_window"]; print_every = CONFIG["print_every"]
    min_dt = 1.0 / max(1e-6, CONFIG["rate_hz"])

    mi = CONFIG["m_idx"]; ms = CONFIG["m_sign"]
    mb = CONFIG["m_bias"]; sc = CONFIG["m_scale"]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    yaw_buf = deque(maxlen=max(1, lp))
    last_send = 0.0; sample_idx = 0

    with serial.Serial(port, baud, timeout=1) as ser:
        ser.reset_input_buffer()
        print(f'Listening {port} @ {baud} → UDP {dest}')
        while True:
            line = ser.readline().decode('utf-8', errors='ignore')
            if not line:
                continue
            vals = parse_vals(line)
            if not vals:
                continue

            # 가속도(XYMU: 0,1,2)
            ax, ay, az = vals[0], vals[1], vals[2]

            # 자력계: 인덱스/부호/바이어스/스케일 적용
            mx_raw, my_raw, mz_raw = vals[mi[0]], vals[mi[1]], vals[mi[2]]
            mx = ((mx_raw - mb[0]) * sc[0]) * ms[0]
            my = ((my_raw - mb[1]) * sc[1]) * ms[1]
            mz = ((mz_raw - mb[2]) * sc[2]) * ms[2]

            # yaw 계산
            yaw_deg = compute_yaw(
                ax, ay, az, mx, my, mz,
                use_nav=CONFIG["use_nav_atan2"],
                yaw_sign=CONFIG["yaw_sign"],
                yaw_off=CONFIG["yaw_offset_deg"],
                decl=CONFIG["declination_deg"]
            )

            # 스무딩 → heading
            yaw_buf.append(yaw_deg)
            heading_deg = sum(yaw_buf)/len(yaw_buf)

            # 전송 주기 제한 후 UDP 송신(JSON)
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
                    print(f"send → {dest}  yaw={yaw_deg:7.2f}°  heading(avg)={heading_deg:7.2f}°")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n종료")
