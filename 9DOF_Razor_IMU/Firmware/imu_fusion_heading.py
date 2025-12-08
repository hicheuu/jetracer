"""
SparkFun 9DoF Razor IMU M0 - 지자기 퓨전 Heading
쿼터니언(자이로) + 지자기(컴퍼스)를 Complementary Filter로 퓨전하여
Drift 없는 안정적인 Heading을 출력합니다.

사용법:
    1. Arduino IDE로 _9DoF_Razor_M0_Firmware.ino 업로드
    2. python imu_fusion_heading.py

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

# Complementary Filter 계수
# ALPHA가 높을수록 자이로(단기) 신뢰, 낮을수록 지자기(장기) 신뢰
ALPHA = 0.98  # 0.95 ~ 0.99 사이 조정

def find_imu_port():
    """사용 가능한 시리얼 포트 목록을 보여주고 선택"""
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

def parse_sparkfun_data(line):
    """
    SparkFun 공식 펌웨어 출력 형식 파싱
    
    config.h 기본 설정 기준:
    time, ax, ay, az, gx, gy, gz, mx, my, mz, heading
    
    또는 쿼터니언 포함 시:
    time, ax, ay, az, gx, gy, gz, mx, my, mz, qw, qx, qy, qz, heading
    """
    try:
        # #XYMU 포맷이면 무시 (기존 펌웨어)
        if line.startswith('#'):
            return None
        
        values = [float(v.strip()) for v in line.split(',')]
        
        # 최소 11개 값 (time + accel + gyro + mag + heading)
        if len(values) >= 11:
            return {
                'time': values[0],
                'ax': values[1],
                'ay': values[2],
                'az': values[3],
                'gx': values[4],
                'gy': values[5],
                'gz': values[6],
                'mx': values[7],
                'my': values[8],
                'mz': values[9],
                'compass_heading': values[10] if len(values) >= 11 else None,
                # 쿼터니언이 있으면
                'qw': values[11] if len(values) >= 15 else None,
                'qx': values[12] if len(values) >= 15 else None,
                'qy': values[13] if len(values) >= 15 else None,
                'qz': values[14] if len(values) >= 15 else None,
            }
        # 값이 적으면 다른 포맷 시도
        elif len(values) >= 4:
            return {
                'raw_values': values,
                'count': len(values)
            }
    except (ValueError, IndexError):
        pass
    
    return None

def quaternion_to_yaw(qw, qx, qy, qz):
    """쿼터니언에서 Yaw(Heading) 각도 계산"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    yaw_deg = math.degrees(yaw)
    if yaw_deg < 0:
        yaw_deg += 360
    
    return yaw_deg

def magnetometer_heading(mx, my):
    """지자기 데이터에서 Heading 계산 (수평 상태 가정)"""
    heading = math.atan2(my, mx)
    heading_deg = math.degrees(heading)
    
    if heading_deg < 0:
        heading_deg += 360
    
    return heading_deg

def complementary_filter(gyro_heading, mag_heading, last_fused, alpha=ALPHA):
    """
    Complementary Filter로 자이로 + 지자기 퓨전
    
    자이로: 빠른 응답, 단기 안정, drift 있음
    지자기: 느린 응답, 장기 안정, drift 없음
    
    fused = α × gyro + (1-α) × mag
    """
    # 각도 차이 계산 (wrap-around 처리)
    diff = mag_heading - gyro_heading
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
    
    # 퓨전
    fused = gyro_heading + (1 - alpha) * diff
    
    # 0~360 범위로 정규화
    fused = fused % 360
    if fused < 0:
        fused += 360
    
    return fused

def draw_compass(heading):
    """간단한 ASCII 나침반"""
    directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
    idx = int((heading + 22.5) % 360 / 45)
    direction = directions[idx]
    
    bar_width = 40
    filled = int((heading / 360) * bar_width)
    bar = '█' * filled + '░' * (bar_width - filled)
    
    return direction, bar

def main():
    print("=" * 70)
    print("  🧭 SparkFun 9DoF Razor IMU M0 - 지자기 퓨전 Heading")
    print("=" * 70)
    print(f"  Complementary Filter α = {ALPHA}")
    print(f"  자이로 신뢰도: {ALPHA*100:.0f}% | 지자기 신뢰도: {(1-ALPHA)*100:.0f}%")
    print("=" * 70)
    
    port = find_imu_port()
    if not port:
        sys.exit(1)
    
    print(f"\n🔗 {port}에 연결 중... (보레이트: {BAUD_RATE})")
    
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2)
        
        print("✅ 연결 성공!")
        ser.reset_input_buffer()
        
        print("\n🧭 Heading 데이터 수신 중... (Ctrl+C로 종료)")
        print("   보드를 Z축 기준으로 회전시켜 보세요!")
        print("-" * 70)
        
        # 상태 변수
        last_gyro_yaw = None
        fused_heading = None
        data_count = 0
        
        while True:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        # 처음 몇 개는 raw 데이터 출력 (디버깅용)
                        if data_count < 3:
                            print(f"📥 Raw: {line}")
                            data_count += 1
                            continue
                        
                        data = parse_sparkfun_data(line)
                        if data:
                            # compass_heading이 있으면 사용
                            if data.get('compass_heading') is not None:
                                mag_heading = data['compass_heading']
                                
                                # 쿼터니언이 있으면 자이로 yaw 계산
                                if data.get('qw') is not None:
                                    gyro_yaw = quaternion_to_yaw(
                                        data['qw'], data['qx'], 
                                        data['qy'], data['qz']
                                    )
                                else:
                                    # 쿼터니언 없으면 지자기만 사용
                                    gyro_yaw = mag_heading
                                
                                # Complementary Filter 퓨전
                                if fused_heading is None:
                                    fused_heading = mag_heading
                                else:
                                    fused_heading = complementary_filter(
                                        fused_heading, mag_heading, fused_heading
                                    )
                                
                                # 변화량 계산
                                if last_gyro_yaw is not None:
                                    delta = fused_heading - last_gyro_yaw
                                    if delta > 180:
                                        delta -= 360
                                    elif delta < -180:
                                        delta += 360
                                    delta_str = f"Δ {delta:+6.1f}°"
                                else:
                                    delta_str = "        "
                                
                                last_gyro_yaw = fused_heading
                                
                                # 나침반 표시
                                direction, bar = draw_compass(fused_heading)
                                
                                print(f"\r  🧭 Fused: {fused_heading:6.1f}° | "
                                      f"Mag: {mag_heading:6.1f}° | "
                                      f"{direction:>2} | [{bar}] | {delta_str}  ", 
                                      end='', flush=True)
                            
                            # raw_values가 있으면 포맷 분석 출력
                            elif data.get('raw_values'):
                                vals = data['raw_values']
                                print(f"\r📊 {data['count']}개 값: {vals[:5]}...", 
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

