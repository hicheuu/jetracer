#!/usr/bin/env python3
"""
9축 Madgwick AHRS Filter for SparkFun 9DoF Razor IMU
- 자력계 캘리브레이션 지원
- drift 없는 정밀 Heading
- 캘리브레이션 값 파일 저장/로드
"""

import serial
import serial.tools.list_ports
import math
import json
import os
import sys
import time
from collections import deque

# ============================================================
# 설정
# ============================================================
SERIAL_BAUD = 115200
CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "mag_calibration.json")

# Madgwick Filter 파라미터
BETA = 0.01  # 필터 게인 (낮을수록 자이로 신뢰, 안정적)
SAMPLE_PERIOD = 1/100  # 100Hz 가정

# 기본 캘리브레이션 값 (캘리브레이션 전)
DEFAULT_CALIBRATION = {
    "mx_offset": 0.0,
    "my_offset": 0.0,
    "mz_offset": 0.0,
    "mx_scale": 1.0,
    "my_scale": 1.0,
    "mz_scale": 1.0,
    "heading_offset": None  # DMP → 자력계 변환 오프셋
}

# ============================================================
# 캘리브레이션 파일 관리
# ============================================================
def load_calibration():
    """캘리브레이션 파일 로드"""
    if os.path.exists(CALIBRATION_FILE):
        try:
            with open(CALIBRATION_FILE, 'r') as f:
                cal = json.load(f)
                print(f"✅ 캘리브레이션 로드됨: {CALIBRATION_FILE}")
                return cal
        except Exception as e:
            print(f"⚠️ 캘리브레이션 로드 실패: {e}")
    print("ℹ️ 기본 캘리브레이션 사용 (캘리브레이션 필요!)")
    return DEFAULT_CALIBRATION.copy()


def save_calibration(cal):
    """캘리브레이션 파일 저장"""
    try:
        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(cal, f, indent=2)
        print(f"✅ 캘리브레이션 저장됨: {CALIBRATION_FILE}")
        return True
    except Exception as e:
        print(f"❌ 캘리브레이션 저장 실패: {e}")
        return False


# ============================================================
# 시리얼 포트 찾기
# ============================================================
def find_serial_port():
    """IMU 시리얼 포트 자동 찾기"""
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if 'ACM' in port.device or 'USB' in port.device or 'usbmodem' in port.device:
            return port.device
        if 'Arduino' in port.description or 'SparkFun' in port.description:
            return port.device
    if ports:
        return ports[0].device
    return None


# ============================================================
# 데이터 파싱
# ============================================================
def parse_imu_data(line):
    """IMU 데이터 파싱 (SparkFun 공식 펌웨어 형식)"""
    try:
        parts = line.split(',')
        if len(parts) >= 15:
            return {
                'timestamp': float(parts[0]),
                'ax': float(parts[1]),
                'ay': float(parts[2]),
                'az': float(parts[3]),
                'gx': float(parts[4]),
                'gy': float(parts[5]),
                'gz': float(parts[6]),
                'mx': float(parts[7]),
                'my': float(parts[8]),
                'mz': float(parts[9]),
                'qw': float(parts[10]),
                'qx': float(parts[11]),
                'qy': float(parts[12]),
                'qz': float(parts[13]),
                'heading': float(parts[14])
            }
    except:
        pass
    return None


# ============================================================
# Madgwick AHRS Filter
# ============================================================
class MadgwickAHRS:
    """9축 Madgwick AHRS Filter"""
    
    def __init__(self, beta=0.1, sample_period=0.01):
        self.beta = beta
        self.sample_period = sample_period
        self.q = [1.0, 0.0, 0.0, 0.0]  # 쿼터니언 [w, x, y, z]
    
    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        """9축 센서 데이터로 쿼터니언 업데이트"""
        q0, q1, q2, q3 = self.q
        dt = self.sample_period
        
        # 자이로 rad/s 변환
        gx = math.radians(gx)
        gy = math.radians(gy)
        gz = math.radians(gz)
        
        # 가속도 정규화
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0:
            return
        ax, ay, az = ax/norm, ay/norm, az/norm
        
        # 자력계 정규화
        norm = math.sqrt(mx*mx + my*my + mz*mz)
        if norm == 0:
            return
        mx, my, mz = mx/norm, my/norm, mz/norm
        
        # 지구 자기장 기준 벡터 계산
        _2q0mx = 2.0 * q0 * mx
        _2q0my = 2.0 * q0 * my
        _2q0mz = 2.0 * q0 * mz
        _2q1mx = 2.0 * q1 * mx
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q0q2 = 2.0 * q0 * q2
        _2q2q3 = 2.0 * q2 * q3
        q0q0 = q0 * q0
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q0q3 = q0 * q3
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q3q3 = q3 * q3
        
        # 자기장 기준 방향
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz
        
        # Gradient descent
        s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        
        # 정규화
        norm = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        if norm > 0:
            s0, s1, s2, s3 = s0/norm, s1/norm, s2/norm, s3/norm
        
        # 쿼터니언 미분
        qDot0 = 0.5 * (-q1*gx - q2*gy - q3*gz) - self.beta*s0
        qDot1 = 0.5 * (q0*gx + q2*gz - q3*gy) - self.beta*s1
        qDot2 = 0.5 * (q0*gy - q1*gz + q3*gx) - self.beta*s2
        qDot3 = 0.5 * (q0*gz + q1*gy - q2*gx) - self.beta*s3
        
        # 적분
        q0 += qDot0 * dt
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt
        
        # 정규화
        norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self.q = [q0/norm, q1/norm, q2/norm, q3/norm]
    
    def get_euler(self):
        """오일러 각도 반환 (roll, pitch, yaw) in degrees"""
        q0, q1, q2, q3 = self.q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (q0*q1 + q2*q3)
        cosr_cosp = 1.0 - 2.0 * (q1*q1 + q2*q2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (q0*q2 - q3*q1)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi/2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (q0*q3 + q1*q2)
        cosy_cosp = 1.0 - 2.0 * (q2*q2 + q3*q3)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def get_yaw(self):
        """Yaw만 반환 (0~360도)"""
        _, _, yaw = self.get_euler()
        if yaw < 0:
            yaw += 360
        return yaw


# ============================================================
# 캘리브레이션 모드
# ============================================================
def run_calibration(ser):
    """자력계 캘리브레이션 수행"""
    print("\n" + "=" * 60)
    print("🧭 자력계 캘리브레이션 모드")
    print("=" * 60)
    print("IMU를 손에 들고 모든 방향으로 천천히 돌리세요!")
    print("  - 8자 모양으로 공중에서 돌리기")
    print("  - 모든 축으로 회전")
    print("  - 뒤집기도 포함")
    print("30초~1분 정도 진행하세요.")
    print("종료하려면 Ctrl+C")
    print("=" * 60 + "\n")
    
    mx_list, my_list, mz_list = [], [], []
    
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                data = parse_imu_data(line)
                if data:
                    mx_list.append(data['mx'])
                    my_list.append(data['my'])
                    mz_list.append(data['mz'])
                    
                    if len(mx_list) % 50 == 0:
                        print(f"샘플 {len(mx_list):5d} | "
                              f"mx: {data['mx']:7.1f} "
                              f"my: {data['my']:7.1f} "
                              f"mz: {data['mz']:7.1f}")
    
    except KeyboardInterrupt:
        print("\n\n" + "=" * 60)
        print("📊 캘리브레이션 계산 중...")
        
        if len(mx_list) < 100:
            print("❌ 데이터가 너무 적습니다. 다시 시도하세요.")
            return None
        
        # Hard Iron 보정 (오프셋)
        mx_offset = (max(mx_list) + min(mx_list)) / 2
        my_offset = (max(my_list) + min(my_list)) / 2
        mz_offset = (max(mz_list) + min(mz_list)) / 2
        
        # Soft Iron 보정 (스케일)
        mx_range = (max(mx_list) - min(mx_list)) / 2
        my_range = (max(my_list) - min(my_list)) / 2
        mz_range = (max(mz_list) - min(mz_list)) / 2
        
        avg_range = (mx_range + my_range + mz_range) / 3
        
        mx_scale = avg_range / mx_range if mx_range > 0 else 1.0
        my_scale = avg_range / my_range if my_range > 0 else 1.0
        mz_scale = avg_range / mz_range if mz_range > 0 else 1.0
        
        cal = {
            "mx_offset": round(mx_offset, 2),
            "my_offset": round(my_offset, 2),
            "mz_offset": round(mz_offset, 2),
            "mx_scale": round(mx_scale, 4),
            "my_scale": round(my_scale, 4),
            "mz_scale": round(mz_scale, 4)
        }
        
        print("=" * 60)
        print("✅ 캘리브레이션 결과:")
        print(f"   MX_OFFSET = {cal['mx_offset']}")
        print(f"   MY_OFFSET = {cal['my_offset']}")
        print(f"   MZ_OFFSET = {cal['mz_offset']}")
        print(f"   MX_SCALE  = {cal['mx_scale']}")
        print(f"   MY_SCALE  = {cal['my_scale']}")
        print(f"   MZ_SCALE  = {cal['mz_scale']}")
        print("=" * 60)
        
        # 저장
        if save_calibration(cal):
            print("✅ 캘리브레이션이 저장되었습니다!")
        
        return cal
    
    return None


# ============================================================
# 메인 Heading 모드
# ============================================================
def run_heading(ser, cal):
    """DMP 쿼터니언 + 자력계 drift 보정"""
    print("\n" + "=" * 60)
    print("🧭 DMP 쿼터니언 + 자력계 Drift 보정")
    print("=" * 60)
    print(f"캘리브레이션: MX_OFF={cal['mx_offset']}, MY_OFF={cal['my_offset']}, MZ_OFF={cal['mz_offset']}")
    print("종료하려면 Ctrl+C")
    print("=" * 60 + "\n")
    
    # DMP Yaw 추적
    prev_dmp_yaw = None
    fused_yaw = None
    
    # 자력계 이동평균 필터 (노이즈 감소)
    mag_heading_buf = deque(maxlen=10)  # 10샘플로 줄임
    
    # Drift 보정 상수
    DRIFT_SLOW = 0.005      # 평상시: 0.5%씩 보정
    DRIFT_FAST = 0.5        # 시작/빠른 회전: 50%씩 빠르게 수렴!
    FAST_ROTATION_THRESHOLD = 3.0  # 3도/샘플 이상이면 빠른 회전
    
    # 빠른 회전 감지용
    fast_rotation_cooldown = 200  # 시작 시 200샘플 동안 빠른 수렴!
    
    # 자력계-DMP 오프셋 (캘리브레이션에서 로드 또는 자동 계산)
    if cal.get('heading_offset') is not None:
        mag_dmp_offset = cal['heading_offset']
        print(f"✅ Heading 오프셋 로드됨: {mag_dmp_offset:.1f}°")
        offset_calibrated = True
    else:
        mag_dmp_offset = None
        offset_calibrated = False
        print("⚠️ Heading 오프셋 미설정 - 캘리브레이션 필요!")
    
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                data = parse_imu_data(line)
                if data:
                    # ========================================
                    # 1. DMP 쿼터니언에서 Yaw 계산 (안정적!)
                    # ========================================
                    qw, qx, qy, qz = data['qw'], data['qx'], data['qy'], data['qz']
                    dmp_yaw = math.degrees(math.atan2(
                        2 * (qw * qz + qx * qy),
                        1 - 2 * (qy * qy + qz * qz)
                    ))
                    if dmp_yaw < 0:
                        dmp_yaw += 360
                    
                    # ========================================
                    # 2. 자력계 Heading (drift 보정용)
                    # ========================================
                    mx = (data['mx'] - cal['mx_offset']) * cal['mx_scale']
                    my = (data['my'] - cal['my_offset']) * cal['my_scale']
                    
                    mag_heading = math.degrees(math.atan2(my, mx))
                    if mag_heading < 0:
                        mag_heading += 360
                    
                    mag_heading_buf.append(mag_heading)
                    mag_avg = sum(mag_heading_buf) / len(mag_heading_buf)
                    
                    # ========================================
                    # 3. DMP 변화량 기반 Fused Yaw
                    # ========================================
                    if prev_dmp_yaw is None:
                        prev_dmp_yaw = dmp_yaw
                        
                        # 오프셋이 캘리브레이션되어 있으면 사용
                        if offset_calibrated:
                            # DMP를 자력계 좌표계로 변환
                            fused_yaw = dmp_yaw - mag_dmp_offset
                            if fused_yaw < 0:
                                fused_yaw += 360
                            elif fused_yaw >= 360:
                                fused_yaw -= 360
                        else:
                            # 캘리브레이션 안 됨 - 첫 자력계 값으로 초기화
                            fused_yaw = mag_avg
                            mag_dmp_offset = dmp_yaw - mag_avg
                    else:
                        # DMP 변화량 계산
                        delta = dmp_yaw - prev_dmp_yaw
                        if delta > 180:
                            delta -= 360
                        elif delta < -180:
                            delta += 360
                        
                        # Fused Yaw 업데이트 (DMP 변화량 적용)
                        fused_yaw += delta
                        
                        # 0~360 범위 유지
                        if fused_yaw < 0:
                            fused_yaw += 360
                        elif fused_yaw >= 360:
                            fused_yaw -= 360
                        
                        # ========================================
                        # 4. 빠른 회전 감지
                        # ========================================
                        if abs(delta) > FAST_ROTATION_THRESHOLD:
                            fast_rotation_cooldown = 100  # 100샘플 동안 빠른 수렴
                        
                        # ========================================
                        # 5. 자력계로 drift 보정 (적응형)
                        # ========================================
                        # 자력계에 오프셋 적용
                        mag_corrected = mag_avg + mag_dmp_offset
                        if mag_corrected < 0:
                            mag_corrected += 360
                        elif mag_corrected >= 360:
                            mag_corrected -= 360
                        
                        # (보정된 자력계와 fused_yaw 차이 계산)
                        mag_diff = mag_corrected - fused_yaw
                        if mag_diff > 180:
                            mag_diff -= 360
                        elif mag_diff < -180:
                            mag_diff += 360
                        
                        # 빠른 회전 후엔 빠르게, 평상시엔 천천히 보정
                        if fast_rotation_cooldown > 0:
                            correction = DRIFT_FAST
                            fast_rotation_cooldown -= 1
                        else:
                            correction = DRIFT_SLOW
                        
                        fused_yaw += mag_diff * correction
                        
                        # 0~360 범위 유지
                        if fused_yaw < 0:
                            fused_yaw += 360
                        elif fused_yaw >= 360:
                            fused_yaw -= 360
                        
                        prev_dmp_yaw = dmp_yaw
                    
                    # ========================================
                    # 6. 절대 방향 출력 (자력계 기준, 0~360°)
                    # ========================================
                    absolute_yaw = fused_yaw
                    
                    # 방위 표시 (8방위)
                    directions = ['N ', 'NE', 'E ', 'SE', 'S ', 'SW', 'W ', 'NW']
                    idx = int((absolute_yaw + 22.5) / 45) % 8
                    direction = directions[idx]
                    
                    # 바 그래프 (0~360°, N이 중앙)
                    # 북쪽(0°)을 중앙에 표시
                    bar_chars = ['░'] * 40
                    bar_pos = int(absolute_yaw / 360 * 40) % 40
                    bar_chars[bar_pos] = '█'
                    bar_chars[20] = '|'  # 중앙 마커 (180°)
                    bar = ''.join(bar_chars)
                    
                    print(f"\r🧭 Heading: {absolute_yaw:5.1f}° [{direction}] W|{bar}|E", end='', flush=True)
    
    except KeyboardInterrupt:
        print("\n\n종료!")


# ============================================================
# 메인
# ============================================================
def main():
    print("=" * 60)
    print("🧭 9DoF Razor IMU - Madgwick AHRS")
    print("=" * 60)
    
    # 시리얼 포트 찾기
    port = find_serial_port()
    if not port:
        print("❌ IMU를 찾을 수 없습니다!")
        sys.exit(1)
    
    print(f"📡 포트: {port}")
    
    # 캘리브레이션 로드
    cal = load_calibration()
    
    # 메뉴
    print("\n" + "-" * 40)
    print("모드 선택:")
    print("  1. Heading 측정")
    print("  2. 자력계 캘리브레이션 (8자 운동)")
    print("  3. Heading 기준점 설정 (북쪽 = 0°)")
    print("-" * 40)
    
    try:
        choice = input("선택 (1, 2, 또는 3): ").strip()
    except:
        choice = "1"
    
    # 시리얼 연결
    try:
        ser = serial.Serial(port, SERIAL_BAUD, timeout=1)
        time.sleep(0.5)  # 연결 안정화
        
        if choice == "2":
            new_cal = run_calibration(ser)
            if new_cal:
                cal = new_cal
                print("\n새 캘리브레이션으로 Heading 측정을 시작하시겠습니까? (y/n)")
                if input().strip().lower() == 'y':
                    run_heading(ser, cal)
        
        elif choice == "3":
            # Heading 기준점 설정
            print("\n" + "=" * 60)
            print("🧭 Heading 기준점 설정")
            print("=" * 60)
            print("IMU를 북쪽(또는 기준 방향)으로 향하게 하세요.")
            print("준비되면 Enter를 누르세요...")
            input()
            
            print("측정 중... (실시간으로 Heading 변화를 표시합니다)")
            dmp_samples = []
            mag_samples = []
            sample_target = 120  # 더 길게 수집해 평균 안정화

            def _bar(angle_deg, width=40):
                bar_chars = ['░'] * width
                pos = int(angle_deg / 360 * width) % width
                bar_chars[pos] = '█'
                bar_chars[width // 2] = '|'  # 180° 마커
                return ''.join(bar_chars)
            
            for idx in range(sample_target):
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                data = parse_imu_data(line)
                if not data:
                    continue

                # DMP Yaw
                qw, qx, qy, qz = data['qw'], data['qx'], data['qy'], data['qz']
                dmp_yaw = math.degrees(math.atan2(
                    2 * (qw * qz + qx * qy),
                    1 - 2 * (qy * qy + qz * qz)
                ))
                if dmp_yaw < 0:
                    dmp_yaw += 360
                dmp_samples.append(dmp_yaw)

                # 자력계 Heading
                mx = (data['mx'] - cal['mx_offset']) * cal['mx_scale']
                my = (data['my'] - cal['my_offset']) * cal['my_scale']
                mag_h = math.degrees(math.atan2(my, mx))
                if mag_h < 0:
                    mag_h += 360
                mag_samples.append(mag_h)

                # 실시간 시각화 (DMP와 자력계 평균을 모두 표시)
                avg_dmp_live = sum(dmp_samples) / len(dmp_samples)
                avg_mag_live = sum(mag_samples) / len(mag_samples)
                progress = (idx + 1) / sample_target * 100
                bar_dmp = _bar(avg_dmp_live)
                bar_mag = _bar(avg_mag_live)
                print(
                    f"\r[{progress:5.1f}%] DMP:{avg_dmp_live:6.2f}° |{bar_dmp}| "
                    f"MAG:{avg_mag_live:6.2f}° |{bar_mag}| Δ={avg_dmp_live-avg_mag_live:+.2f}°",
                    end='',
                    flush=True,
                )
            
            print()  # 줄바꿈

            if len(dmp_samples) > 10:
                avg_dmp = sum(dmp_samples) / len(dmp_samples)
                avg_mag = sum(mag_samples) / len(mag_samples)
                
                # 오프셋 계산: DMP - 0° (현재 방향이 북쪽이 되도록)
                # heading_offset = DMP값 - 원하는_heading
                # 현재 방향이 0°가 되길 원하므로: heading_offset = avg_dmp - 0 = avg_dmp
                heading_offset = avg_dmp
                
                cal['heading_offset'] = round(heading_offset, 2)
                
                print(f"\n✅ Heading 오프셋 설정됨: {heading_offset:.1f}°")
                print(f"   (DMP: {avg_dmp:.1f}°, 자력계: {avg_mag:.1f}°)")
                
                if save_calibration(cal):
                    print("✅ 저장 완료!")
                
                print("\nHeading 측정을 시작하시겠습니까? (y/n)")
                if input().strip().lower() == 'y':
                    run_heading(ser, cal)
            else:
                print("❌ 데이터 수집 실패")
        
        else:
            run_heading(ser, cal)
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"❌ 시리얼 에러: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

