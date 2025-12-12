import serial
import serial.tools.list_ports
import time
import math


BAUD_RATE = 115200


# -----------------------------
# 포트 자동 선택
# -----------------------------
def find_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise Exception("No IMU detected")

    print("\n📡 Available Ports")
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device} - {p.description}")

    if len(ports) == 1:
        print(f"Auto-selected {ports[0].device}")
        return ports[0].device

    idx = int(input("Select port number: "))
    return ports[idx].device


# -----------------------------
# Roll / Pitch 계산 함수
# -----------------------------
def accel_to_rp(ax, ay, az):
    # accel-based roll, pitch (in degrees)
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return math.degrees(roll), math.degrees(pitch)


def quat_to_rp(qw, qx, qy, qz):
    # quaternion-based roll, pitch (in degrees)

    # roll (x축)
    sinr_cosp = 2 * (qw*qx + qy*qz)
    cosr_cosp = 1 - 2 * (qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y축)
    sinp = 2 * (qw*qy - qz*qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi/2, sinp)
    else:
        pitch = math.asin(sinp)

    return math.degrees(roll), math.degrees(pitch)


# -----------------------------
# 메인 루프
# -----------------------------
def main():
    port = find_port()
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    time.sleep(2)

    print("\n📡 Roll/Pitch Accuracy Test Running...")
    print("보드를 천천히 좌/우, 앞/뒤로 기울여보세요.")
    print("Ctrl+C로 종료\n")

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            parts = line.split(",")

            if len(parts) < 14:
                continue

            ax = float(parts[1])
            ay = float(parts[2])
            az = float(parts[3])

            qw = float(parts[10])
            qx = float(parts[11])
            qy = float(parts[12])
            qz = float(parts[13])

            # 계산
            a_roll, a_pitch = accel_to_rp(ax, ay, az)
            q_roll, q_pitch = quat_to_rp(qw, qx, qy, qz)

            print(
                f"\rACC RP = R:{a_roll:6.2f}°  P:{a_pitch:6.2f}°   |   "
                f"QUAT RP = R:{q_roll:6.2f}°  P:{q_pitch:6.2f}°",
                end="",
                flush=True
            )

    except KeyboardInterrupt:
        print("\n\n종료합니다.")
        ser.close()


if __name__ == "__main__":
    main()
