#!/usr/bin/env python3
"""
Keyboard Drive with IMU Yaw Hold
- imu_madgwick.py의 DMP + 자력계 drift 보정 로직 그대로 사용
- keyboard_drive.py의 키보드 제어 방식 사용
- P, D 제어만 (I 제어 없음)
- throttle PID 없음 (단순 증감)
"""

import sys
import math
import threading
import time
import json
import os
import select
from collections import deque

# ============================================================
# 키보드 입력 (keyboard_drive.py에서 가져옴)
# ============================================================
try:
    import msvcrt

    def _getch():
        try:
            return msvcrt.getch().decode("utf-8").lower()
        except UnicodeDecodeError:
            return ""
except ImportError:
    import termios
    import tty

    def _getch_blocking():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1).lower()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch


# 비블로킹 키보드 리더
class KeyboardReader:
    def __init__(self):
        self.running = False
        self.last_key = None
        self.lock = threading.Lock()
        self.thread = None
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)

    def _read_loop(self):
        try:
            tty.setcbreak(self.fd)
            while self.running:
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    key = sys.stdin.read(1).lower()
                    with self.lock:
                        self.last_key = key
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        except:
            pass

    def get_key(self):
        with self.lock:
            key = self.last_key
            self.last_key = None
            return key


# ============================================================
# IMU (imu_madgwick.py에서 그대로 가져옴)
# ============================================================
import serial
import serial.tools.list_ports

SERIAL_BAUD = 115200
# 캘리브레이션 파일은 sensors 디렉토리에 저장
CALIBRATION_FILE = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), 
    "sensors", 
    "mag_calibration.json"
)

DEFAULT_CALIBRATION = {
    "mx_offset": 0.0,
    "my_offset": 0.0,
    "mz_offset": 0.0,
    "mx_scale": 1.0,
    "my_scale": 1.0,
    "mz_scale": 1.0,
    "heading_offset": None
}


def load_calibration():
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


def find_serial_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if 'ACM' in port.device or 'USB' in port.device or 'usbmodem' in port.device:
            return port.device
        if 'Arduino' in port.description or 'SparkFun' in port.description:
            return port.device
    if ports:
        return ports[0].device
    return None


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
# Yaw Hold Controller (imu_madgwick.py의 run_heading 로직 사용)
# ============================================================
class YawHoldController:
    """
    imu_madgwick.py의 DMP + 자력계 drift 보정 로직을 그대로 사용
    """

    def __init__(self, cal, invert_steering=True):
        self.cal = cal
        self.invert_steering = invert_steering

        # imu_madgwick.py의 상수들
        self.DRIFT_SLOW = 0.015      # 평상시: 0.5%씩 보정
        self.DRIFT_FAST = 0.7        # 시작/빠른 회전: 50%씩 빠르게 수렴
        self.FAST_ROTATION_THRESHOLD = 2.5  # 3도/샘플 이상이면 빠른 회전

        # 상태
        self.target_yaw = None
        self.fused_yaw = None
        self.lock = threading.Lock()

        # IMU
        self.running = False
        self.ser = None
        self.thread = None

        # 초기화 완료 여부
        self.initialized = False
        self.warmup_count = 0
        self.warmup_target = 100  # 약 2초

    def _read_loop(self):
        """imu_madgwick.py의 run_heading 로직 그대로"""
        cal = self.cal

        # Heading 오프셋
        if cal.get('heading_offset') is not None:
            mag_dmp_offset = cal['heading_offset']
            offset_calibrated = True
        else:
            mag_dmp_offset = None
            offset_calibrated = False

        # DMP Yaw 추적
        prev_dmp_yaw = None
        fused_yaw = None

        # 자력계 이동평균 필터
        mag_heading_buf = deque(maxlen=5)

        # 빠른 회전 감지용
        fast_rotation_cooldown = 300  # 시작 시 200샘플 동안 빠른 수렴

        while self.running:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                data = parse_imu_data(line)
                if not data:
                    continue

                # ========================================
                # 1. DMP 쿼터니언에서 Yaw 계산
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

                    if offset_calibrated:
                        fused_yaw = dmp_yaw - mag_dmp_offset
                        if fused_yaw < 0:
                            fused_yaw += 360
                        elif fused_yaw >= 360:
                            fused_yaw -= 360
                    else:
                        fused_yaw = mag_avg
                        mag_dmp_offset = dmp_yaw - mag_avg
                else:
                    # DMP 변화량 계산
                    delta = dmp_yaw - prev_dmp_yaw
                    if delta > 180:
                        delta -= 360
                    elif delta < -180:
                        delta += 360

                    # Fused Yaw 업데이트
                    fused_yaw += delta

                    # 0~360 범위 유지
                    if fused_yaw < 0:
                        fused_yaw += 360
                    elif fused_yaw >= 360:
                        fused_yaw -= 360

                    # ========================================
                    # 4. 빠른 회전 감지
                    # ========================================
                    if abs(delta) > self.FAST_ROTATION_THRESHOLD:
                        fast_rotation_cooldown = 100

                    # ========================================
                    # 5. 자력계로 drift 보정
                    # ========================================
                    mag_corrected = mag_avg + mag_dmp_offset
                    if mag_corrected < 0:
                        mag_corrected += 360
                    elif mag_corrected >= 360:
                        mag_corrected -= 360

                    mag_diff = mag_corrected - fused_yaw
                    if mag_diff > 180:
                        mag_diff -= 360
                    elif mag_diff < -180:
                        mag_diff += 360

                    # 빠른 회전 후엔 빠르게, 평상시엔 천천히 보정
                    if fast_rotation_cooldown > 0:
                        correction = self.DRIFT_FAST
                        fast_rotation_cooldown -= 1
                    else:
                        correction = self.DRIFT_SLOW

                    fused_yaw += mag_diff * correction

                    # 0~360 범위 유지
                    if fused_yaw < 0:
                        fused_yaw += 360
                    elif fused_yaw >= 360:
                        fused_yaw -= 360

                    prev_dmp_yaw = dmp_yaw

                # 상태 업데이트
                with self.lock:
                    self.fused_yaw = fused_yaw

                    # Warm-up
                    if not self.initialized:
                        self.warmup_count += 1
                        if self.warmup_count >= self.warmup_target:
                            self.target_yaw = fused_yaw
                            self.initialized = True
                            print(f"\n✅ 초기화 완료! 목표 yaw: {fused_yaw:.1f}°")

            except Exception as e:
                time.sleep(0.01)

    def start(self):
        port = find_serial_port()
        if not port:
            print("❌ IMU 포트를 찾을 수 없습니다!")
            return False

        try:
            self.ser = serial.Serial(port, SERIAL_BAUD, timeout=0.1)
            time.sleep(0.5)
            print(f"📡 IMU 연결됨: {port}")
        except Exception as e:
            print(f"❌ IMU 연결 실패: {e}")
            return False

        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.ser:
            self.ser.close()

    def is_ready(self):
        with self.lock:
            return self.initialized

    def get_warmup_progress(self):
        with self.lock:
            return min(100, self.warmup_count * 100 // self.warmup_target)

    def get_error(self):
        """현재 오차 반환 (-180 ~ 180)"""
        with self.lock:
            if self.target_yaw is None or self.fused_yaw is None:
                return 0.0
            error = self.target_yaw - self.fused_yaw
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360
            return error

    def get_yaw(self):
        with self.lock:
            return self.fused_yaw if self.fused_yaw is not None else 0.0

    def get_target(self):
        with self.lock:
            return self.target_yaw

    def adjust_target_yaw(self, delta):
        with self.lock:
            if self.target_yaw is not None:
                self.target_yaw += delta
                if self.target_yaw < 0:
                    self.target_yaw += 360
                elif self.target_yaw >= 360:
                    self.target_yaw -= 360

    def reset_target_yaw(self):
        with self.lock:
            if self.fused_yaw is not None:
                self.target_yaw = self.fused_yaw
                print(f"\n🎯 목표 재설정: {self.fused_yaw:.1f}°")


# ============================================================
# PD 컨트롤러 (I 제어 없음)
# ============================================================
class PDController:
    def __init__(self, kp=0.015, kd=0.008, max_output=1.0, deadband=2.0, max_error=45.0):
        self.kp = kp
        self.kd = kd
        self.max_output = max_output
        self.deadband = deadband
        self.max_error = max_error  # 오차 클램핑 (180도 경계 문제 완화)
        self.prev_error = 0.0
        self.prev_output = 0.0

    def reset(self):
        self.prev_error = 0.0
        self.prev_output = 0.0

    def compute(self, error, dt=0.02):
        # 오차 클램핑 (180도 경계 문제 완화)
        clamped_error = error
        if clamped_error > self.max_error:
            clamped_error = self.max_error
        elif clamped_error < -self.max_error:
            clamped_error = -self.max_error

        # Deadband
        if abs(clamped_error) < self.deadband:
            output = self.prev_output * 0.9
            self.prev_output = output
            return output

        # P
        p_term = self.kp * clamped_error

        # D (클램핑된 오차 사용)
        d_term = self.kd * (clamped_error - self.prev_error) / dt
        self.prev_error = clamped_error

        # 합산
        output = p_term + d_term

        # 클램프
        output = max(-self.max_output, min(self.max_output, output))

        # Rate limit (더 강하게)
        max_change = 0.03  # 0.05 → 0.03
        delta = output - self.prev_output
        if delta > max_change:
            output = self.prev_output + max_change
        elif delta < -max_change:
            output = self.prev_output - max_change

        self.prev_output = output
        return output


# ============================================================
# 메인
# ============================================================
def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def main():
    # 설정
    THROTTLE_STEP = 0.005
    MAX_THROTTLE = 0.2
    YAW_STEP = 15.0
    KP = 0.015  # 감소 (0.02 → 0.015)
    KD = 0.008  # 감소 (0.01 → 0.008)
    DEADBAND = 2.0
    MAX_ERROR = 45.0  # 오차 클램핑
    INVERT_STEERING = False  # False가 올바른 방향

    # JetRacer
    try:
        from jetracer.nvidia_racecar import NvidiaRacecar
        car = NvidiaRacecar()
    except ImportError:
        print("⚠️ JetRacer not found, using mock car")
        class MockCar:
            throttle = 0.0
            steering = 0.0
        car = MockCar()

    # 캘리브레이션 로드
    cal = load_calibration()

    # Yaw Hold Controller
    yaw_ctrl = YawHoldController(cal, invert_steering=INVERT_STEERING)

    # PD Controller
    pd = PDController(kp=KP, kd=KD, deadband=DEADBAND, max_error=MAX_ERROR)

    # 키보드
    keyboard = KeyboardReader()

    print("=" * 50)
    print("🚗 Keyboard Yaw Hold")
    print("=" * 50)
    print("w/s: throttle ±, a/d: yaw ±, r: reset, c: set target, i: invert, q: quit")
    print(f"throttle_step={THROTTLE_STEP}, max={MAX_THROTTLE}")
    print(f"PD: Kp={KP}, Kd={KD}, deadband={DEADBAND}°, max_error={MAX_ERROR}°")
    print("=" * 50)

    if not yaw_ctrl.start():
        return

    keyboard.start()

    # Warm-up 대기
    print("\n⏳ IMU 초기화 중...")
    while not yaw_ctrl.is_ready():
        progress = yaw_ctrl.get_warmup_progress()
        print(f"\r   {progress}%   ", end='', flush=True)
        time.sleep(0.1)

    print("\n" + "=" * 50)
    print("✅ 준비 완료!")
    print("=" * 50)

    throttle = 0.0
    car.throttle = 0.0
    car.steering = 0.0

    try:
        while True:
            key = keyboard.get_key()

            if key == 'w':
                throttle = clamp(throttle + THROTTLE_STEP, 0, MAX_THROTTLE)
            elif key == 's':
                throttle = clamp(throttle - THROTTLE_STEP, 0, MAX_THROTTLE)
            elif key == 'a':
                yaw_ctrl.adjust_target_yaw(YAW_STEP)
            elif key == 'd':
                yaw_ctrl.adjust_target_yaw(-YAW_STEP)
            elif key == 'r':
                throttle = 0.0
                yaw_ctrl.reset_target_yaw()
                pd.reset()
            elif key == 'c':
                yaw_ctrl.reset_target_yaw()
                pd.reset()
            elif key == 'i':
                yaw_ctrl.invert_steering = not yaw_ctrl.invert_steering
                print(f"\n🔄 invert={yaw_ctrl.invert_steering}")
            elif key == 'q':
                break

            # 오차 계산
            error = yaw_ctrl.get_error()
            yaw = yaw_ctrl.get_yaw()

            # PD steering
            steering = pd.compute(error)
            if yaw_ctrl.invert_steering:
                steering = -steering

            # 차량 제어
            if throttle > 0:
                throttle_cmd = ESC_NEUTRAL + throttle * (1.0 - ESC_NEUTRAL) * THROTTLE_SCALE
            elif throttle < 0:
                throttle_cmd = REVERSE_START + throttle * (1.0 - abs(REVERSE_START)) * THROTTLE_SCALE
            else:
                throttle_cmd = ESC_NEUTRAL

            # clamp
            throttle_cmd = max(-1.0, min(1.0, throttle_cmd))

            car.throttle = throttle_cmd
            car.steering = steering

            # 로그 (error, yaw만)
            print(f"\ryaw={yaw:5.1f}° err={error:+5.1f}°   ", end='', flush=True)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\nCtrl+C")
    finally:
        car.throttle = 0.0
        car.steering = 0.0
        yaw_ctrl.stop()
        keyboard.stop()
        print("\n🛑 정지")


if __name__ == "__main__":
    main()
