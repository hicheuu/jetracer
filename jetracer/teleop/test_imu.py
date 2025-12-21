import serial
import time
import threading
import socket
import struct

# ================= 설정 구간 =================
SERIAL_PORT = '/dev/ttyACM1'  # 제트레이서 IMU 포트 (확인 후 변경)
BAUD_RATE = 115200
SERVER_IP = "192.168.0.11"    # 서버 IP 주소 입력
SERVER_PORT = 5560            # 서버 포트 번호
SEND_RATE = 30                # 전송 주파수 (Hz)
# ============================================

class IMUReader:
    def __init__(self, port, baud):
        self.ser = None
        self.gyro_z = 0.0  # 공유 변수 (최신 Gyro Z 값)
        self.running = True
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            print(f"[IMU] {port} 포트 연결 성공")
        except Exception as e:
            print(f"[IMU] 연결 실패: {e}")
            self.running = False

        # 백그라운드 스레드 시작 (데이터 읽기 전용)
        self.thread = threading.Thread(target=self._update_loop)
        self.thread.daemon = True
        self.thread.start()

    def _update_loop(self):
        """시리얼 데이터를 최대한 빠르게 계속 읽어옴"""
        while self.running and self.ser:
            try:
                if self.ser.in_waiting > 0:
                    # 데이터 읽기 및 디코딩
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # 파싱: #XYMU=...# 형식 확인
                    if line.startswith("#XYMU=") and line.endswith("#"):
                        # 앞뒤 태그 제거 (#XYMU= 와 #)
                        content = line.replace("#XYMU=", "").replace("#", "")
                        parts = content.split(',')
                        
                        # 데이터 개수가 10개 이상인지 확인
                        if len(parts) >= 10:
                            # 10번째 값(인덱스 9)이 Gyro Z
                            self.gyro_z = float(parts[9])
                            
            except Exception:
                pass  # 파싱 에러나면 무시하고 다음 데이터 읽음

    def get_gyro_z(self):
        """현재 저장된 최신 Gyro Z 값 반환"""
        return self.gyro_z

def main():
    # UDP 소켓 생성
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # IMU 리더 시작
    imu = IMUReader(SERIAL_PORT, BAUD_RATE)
    
    print(f"[Main] Gyro Z 데이터 전송 시작 (목표: {SEND_RATE}Hz)")
    
    try:
        while True:
            start_time = time.time()
            
            # 1. 최신 Gyro Z 값 가져오기 (스레드에서 갱신 중)
            current_gyro_z = imu.get_gyro_z()
            
            # 2. 패킹 (float 1개 = 4바이트)
            # 서버에서 C언어 구조체나 Python struct.unpack('f')로 풀면 됨
            packet = struct.pack('f', current_gyro_z)
            
            # 3. 전송
            sock.sendto(packet, (SERVER_IP, SERVER_PORT))
            
            # 4. Hz 맞추기 (30Hz = 약 0.033초 주기)
            elapsed = time.time() - start_time
            sleep_time = (1.0 / SEND_RATE) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n종료합니다.")
        imu.running = False
        sock.close()

if __name__ == "__main__":
    main()