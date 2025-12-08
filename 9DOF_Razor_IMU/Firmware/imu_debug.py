"""
IMU 디버깅용 - 시리얼 포트에서 뭐가 들어오는지 확인
"""

import serial
import serial.tools.list_ports
import sys
import time

BAUD_RATE = 115200

def find_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("❌ 포트 없음")
        return None
    print(f"✅ 포트 발견: {ports[0].device}")
    return ports[0].device

def main():
    print("=" * 50)
    print("  🔍 IMU 디버깅 도구")
    print("=" * 50)
    
    port = find_port()
    if not port:
        sys.exit(1)
    
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.5)
        print(f"✅ {port} 연결됨")
        time.sleep(2)
        
        print("\n📊 10초간 데이터 수신 테스트...")
        print("-" * 50)
        
        start = time.time()
        count = 0
        
        # 스페이스 보내서 로깅 활성화 시도
        print("📤 스페이스 전송 (로깅 토글)")
        ser.write(b' ')
        time.sleep(0.2)
        
        while time.time() - start < 10:
            if ser.in_waiting:
                try:
                    data = ser.read(ser.in_waiting)
                    text = data.decode('utf-8', errors='replace')
                    print(f"📥 [{count}] {repr(text)}")
                    count += 1
                except Exception as e:
                    print(f"❌ 에러: {e}")
            else:
                # 1초마다 상태 표시
                elapsed = int(time.time() - start)
                print(f"\r⏳ {elapsed}초... (데이터 없음)", end='', flush=True)
                time.sleep(0.5)
        
        print(f"\n\n📊 결과: {count}개 데이터 수신")
        
        if count == 0:
            print("\n🔧 문제 해결 방법:")
            print("   1. 펌웨어가 업로드 되어있는지 확인")
            print("   2. Arduino IDE 시리얼 모니터로 테스트")
            print("   3. 보드 리셋 버튼 눌러보기")
            print("   4. USB 케이블 다시 연결")
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"❌ 시리얼 오류: {e}")
        print("   → 다른 프로그램이 COM3을 사용 중일 수 있음")
        print("   → Arduino IDE 시리얼 모니터 닫기")
        sys.exit(1)

if __name__ == "__main__":
    main()

