"""
SparkFun 9DoF Razor IMU M0 - FULL 데이터 출력 + CSV 기록 Python 스크립트
모든 출력 필드를 자동 감지하고 보기 좋게 출력하며,
동시에 IMU 보정을 위한 CSV 파일로 저장합니다.
"""

import serial
import serial.tools.list_ports
import sys
import time
import datetime
import csv

BAUD_RATE = 115200


# -------------------------------------------
# 포트 자동 검색
# -------------------------------------------
def find_imu_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("❌ 시리얼 포트 없음.")
        return None

    print("\n📡 사용 가능한 시리얼 포트:")
    print("-" * 40)
    for i, port in enumerate(ports):
        print(f"[{i}] {port.device} - {port.description}")
    print("-" * 40)

    if len(ports) == 1:
        print(f"✅ 자동 선택: {ports[0].device}")
        return ports[0].device

    try:
        idx = int(input("포트 번호 선택: "))
        return ports[idx].device
    except:
        return None


# -------------------------------------------
# CSV 파싱
# -------------------------------------------
def parse_imu_data(line):
    try:
        values = [float(v.strip()) for v in line.split(',')]
        return values
    except:
        return None


# -------------------------------------------
# 실시간 출력
# -------------------------------------------
def print_full_output(values):
    N = len(values)

    if N == 15:
        print(
            f"\r⏱ time={values[0]:.0f}ms | "
            f"ACC[{values[1]:.3f}, {values[2]:.3f}, {values[3]:.3f}] | "
            f"GYR[{values[4]:.2f}, {values[5]:.2f}, {values[6]:.2f}] | "
            f"MAG[{values[7]:.1f}, {values[8]:.1f}, {values[9]:.1f}] | "
            f"Q[{values[10]:.4f}, {values[11]:.4f}, {values[12]:.4f}, {values[13]:.4f}] | "
            f"HDG={values[14]:.2f}°",
            end="",
            flush=True
        )
    else:
        print(f"\r[{N} values] {values}", end="", flush=True)


# -------------------------------------------
# 메인 루프
# -------------------------------------------
def main():
    print("=" * 60)
    print("🎯 SparkFun 9DoF Razor IMU M0 - FULL 데이터 리더 + CSV 기록")
    print("=" * 60)

    port = find_imu_port()
    if not port:
        sys.exit(1)

    print(f"\n🔗 {port} 연결 중... ({BAUD_RATE} baud)")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2)

        print("✅ 연결 성공")

        # -------------------------------
        # CSV 파일 준비
        # -------------------------------
        now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"imu_log_{now}.csv"
        csv_file = open(filename, "w", newline="")
        csv_writer = csv.writer(csv_file)

        # CSV 헤더
        header = [
            "time_ms",
            "ax", "ay", "az",
            "gx", "gy", "gz",
            "mx", "my", "mz",
            "qw", "qx", "qy", "qz",
            "heading"
        ]
        csv_writer.writerow(header)

        print(f"💾 CSV 기록 시작 → {filename}")
        print("📡 데이터 수신 중... (Ctrl+C 종료)\n")

        # -------------------------------
        # 수신 루프
        # -------------------------------
        while True:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                values = parse_imu_data(line)

                if values and len(values) == 15:
                    print_full_output(values)

                    # CSV에 저장
                    csv_writer.writerow(values)

                else:
                    # CSV형태가 아닌 메시지
                    print(f"\n📝 메시지: {line}")

    except KeyboardInterrupt:
        print("\n\n👋 종료합니다.")
    except serial.SerialException as e:
        print(f"\n❌ 시리얼 오류: {e}")
    finally:
        try:
            csv_file.close()
            print(f"💾 CSV 파일 저장 완료: {filename}")
        except:
            pass


if __name__ == "__main__":
    main()
