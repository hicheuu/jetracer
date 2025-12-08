"""
SparkFun 9DoF Razor IMU M0 - 데이터 확인용 Python 스크립트
시리얼 포트로 IMU 데이터를 읽어와서 출력합니다.

사용법:
    python imu_reader.py
    
필요 패키지:
    pip install pyserial
"""

import serial
import serial.tools.list_ports
import sys
import time

# 설정 (config.h와 동일하게)
BAUD_RATE = 115200

def find_imu_port():
    """사용 가능한 시리얼 포트 목록을 보여주고 선택하게 함"""
    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        print("❌ 연결된 시리얼 포트가 없습니다!")
        print("   - USB 케이블이 연결되어 있는지 확인하세요")
        print("   - 드라이버가 설치되어 있는지 확인하세요")
        return None
    
    print("\n📡 사용 가능한 시리얼 포트:")
    print("-" * 50)
    for i, port in enumerate(ports):
        print(f"  [{i}] {port.device} - {port.description}")
    print("-" * 50)
    
    if len(ports) == 1:
        print(f"✅ 자동 선택: {ports[0].device}")
        return ports[0].device
    
    try:
        choice = int(input("포트 번호를 선택하세요: "))
        return ports[choice].device
    except (ValueError, IndexError):
        print("❌ 잘못된 선택입니다.")
        return None

def parse_imu_data(line):
    """IMU 데이터 라인을 파싱"""
    try:
        values = [float(v.strip()) for v in line.split(',')]
        return values
    except ValueError:
        return None

def print_imu_data(values):
    """IMU 데이터를 보기 좋게 출력"""
    if not values:
        return
    
    # 기본 설정 기준 (config.h): time, ax, ay, az, gx, gy, gz, mx, my, mz
    if len(values) >= 10:
        print(f"\r⏱️  Time: {values[0]:>8.0f}ms | "
              f"🔵 Accel: X={values[1]:>7.3f} Y={values[2]:>7.3f} Z={values[3]:>7.3f} g | "
              f"🟢 Gyro: X={values[4]:>8.2f} Y={values[5]:>8.2f} Z={values[6]:>8.2f} dps | "
              f"🔴 Mag: X={values[7]:>7.1f} Y={values[8]:>7.1f} Z={values[9]:>7.1f} uT",
              end='', flush=True)
    elif len(values) >= 7:
        print(f"\r⏱️  Time: {values[0]:>8.0f}ms | "
              f"🔵 Accel: X={values[1]:>7.3f} Y={values[2]:>7.3f} Z={values[3]:>7.3f} g | "
              f"🟢 Gyro: X={values[4]:>8.2f} Y={values[5]:>8.2f} Z={values[6]:>8.2f} dps",
              end='', flush=True)
    elif len(values) >= 4:
        print(f"\r⏱️  Time: {values[0]:>8.0f}ms | "
              f"🔵 Accel: X={values[1]:>7.3f} Y={values[2]:>7.3f} Z={values[3]:>7.3f} g",
              end='', flush=True)
    else:
        print(f"\r📊 Data: {values}", end='', flush=True)

def main():
    print("=" * 60)
    print("  🎯 SparkFun 9DoF Razor IMU M0 - 데이터 확인 도구")
    print("=" * 60)
    
    # 포트 찾기
    port = find_imu_port()
    if not port:
        sys.exit(1)
    
    print(f"\n🔗 {port}에 연결 중... (보레이트: {BAUD_RATE})")
    
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2)  # 아두이노 리셋 대기
        
        print("✅ 연결 성공!")
        print("\n📊 IMU 데이터 수신 중... (Ctrl+C로 종료)\n")
        print("-" * 100)
        
        error_count = 0
        success_count = 0
        
        while True:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        values = parse_imu_data(line)
                        if values:
                            print_imu_data(values)
                            success_count += 1
                            error_count = 0
                        else:
                            # 설정 메시지 등 출력
                            print(f"\n📝 메시지: {line}")
                except UnicodeDecodeError:
                    error_count += 1
                    if error_count > 10:
                        print("\n⚠️  디코딩 오류가 계속됩니다. 보레이트를 확인하세요.")
                        error_count = 0
                        
    except serial.SerialException as e:
        print(f"\n❌ 시리얼 오류: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print(f"\n\n👋 종료합니다. (총 {success_count}개 데이터 수신)")
        ser.close()
        sys.exit(0)

if __name__ == "__main__":
    main()

