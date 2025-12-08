"""
SparkFun 9DoF Razor IMU M0 - Heading/Yaw 확인용 Python 스크립트
쿼터니언에서 Yaw(Heading)를 계산해서 보여줍니다.

사용법:
    python imu_heading.py
    
필요 패키지:
    pip install pyserial
"""

import serial
import serial.tools.list_ports
import sys
import time
import math

# 설정
BAUD_RATE = 115200

# 스케일 보정 (180도 돌렸을 때 ~151.5도로 측정되어 보정)
# 실측: 180도 회전 → 약 151.5도 측정
# 게인 = 180 / 151.5 ≈ 1.19
SCALE_FACTOR = 180.0 / 151.5  # ≈ 1.188

def find_imu_port():
    """사용 가능한 시리얼 포트 목록을 보여주고 선택하게 함"""
    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        print("❌ 연결된 시리얼 포트가 없습니다!")
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

def quaternion_to_yaw(qw, qx, qy, qz):
    """쿼터니언에서 Yaw(Heading) 각도 계산"""
    # Yaw (Z축 회전)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    # 라디안 → 도 변환, 0~360 범위로
    yaw_deg = math.degrees(yaw)
    if yaw_deg < 0:
        yaw_deg += 360
    
    return yaw_deg

def draw_compass(heading):
    """간단한 ASCII 나침반 그리기"""
    directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
    idx = int((heading + 22.5) % 360 / 45)
    direction = directions[idx]
    
    # 진행 바 (0~360도)
    bar_width = 40
    filled = int((heading / 360) * bar_width)
    bar = '█' * filled + '░' * (bar_width - filled)
    
    return direction, bar

def parse_xymu_data(line):
    """
    #XYMU=ax,ay,az,qw,qx,qy,qz,gx,gy,gz# 형식 파싱
    """
    if not line.startswith('#XYMU=') or not line.endswith('#'):
        return None
    
    try:
        # #XYMU= 와 # 제거
        data_str = line[6:-1]
        values = [float(v.strip()) for v in data_str.split(',')]
        
        if len(values) >= 7:
            # 인덱스 3,4,5,6이 쿼터니언 (qw, qx, qy, qz)
            return {
                'ax': values[0],
                'ay': values[1],
                'az': values[2],
                'qw': values[3],
                'qx': values[4],
                'qy': values[5],
                'qz': values[6],
                'gx': values[7] if len(values) > 7 else 0,
                'gy': values[8] if len(values) > 8 else 0,
                'gz': values[9] if len(values) > 9 else 0,
            }
    except (ValueError, IndexError):
        pass
    
    return None

def main():
    print("=" * 60)
    print("  🧭 SparkFun 9DoF Razor IMU M0 - Heading 확인 도구")
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
        ser.reset_input_buffer()
        
        print("\n🧭 Yaw(Heading) 데이터 수신 중... (Ctrl+C로 종료)")
        print(f"   스케일 보정: x{SCALE_FACTOR:.3f}")
        print("   보드를 Z축 기준으로 회전시켜 보세요!")
        print("=" * 70)
        
        last_raw_yaw = None
        corrected_yaw = None
        
        while True:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        data = parse_xymu_data(line)
                        if data:
                            # 쿼터니언에서 Raw Yaw 계산
                            raw_yaw = quaternion_to_yaw(
                                data['qw'], data['qx'], data['qy'], data['qz']
                            )
                            
                            # 변화량 계산 및 스케일 보정
                            if last_raw_yaw is not None:
                                # Raw 변화량
                                delta_raw = raw_yaw - last_raw_yaw
                                if delta_raw > 180:
                                    delta_raw -= 360
                                elif delta_raw < -180:
                                    delta_raw += 360
                                
                                # 스케일 보정 적용
                                delta_corrected = delta_raw * SCALE_FACTOR
                                
                                # 보정된 Yaw 누적
                                corrected_yaw = (corrected_yaw + delta_corrected) % 360
                                
                                delta_str = f"Δ {delta_corrected:+6.1f}°"
                            else:
                                # 첫 번째 값은 그대로 사용
                                corrected_yaw = raw_yaw
                                delta_str = "        "
                            
                            last_raw_yaw = raw_yaw
                            
                            # 나침반 방향 (보정된 값 사용)
                            direction, bar = draw_compass(corrected_yaw)
                            
                            print(f"\r  🧭 Yaw: {corrected_yaw:6.1f}° | {direction:>2} | [{bar}] | {delta_str}  ", 
                                  end='', flush=True)
                                
                except UnicodeDecodeError:
                    pass
                    
    except serial.SerialException as e:
        print(f"\n❌ 시리얼 오류: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print(f"\n\n👋 종료합니다.")
        ser.close()
        sys.exit(0)

if __name__ == "__main__":
    main()
