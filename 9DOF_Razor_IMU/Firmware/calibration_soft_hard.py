import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import threading
import time
import json  # 파일 저장을 위한 라이브러리 추가
import numpy as np  # 계산 편의를 위해 numpy 사용 (없으면 pip install numpy 필요)

# 데이터 저장을 위한 리스트
mx_list = []
my_list = []
running = True  # 스레드 제어용 플래그

# ---- 시리얼 포트 자동 탐색 ----
def find_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise Exception("No IMU detected")
    print(f"Connecting to {ports[0].device}...")
    return ports[0].device

# ---- 시리얼 읽기 스레드 ----
def serial_thread(port):
    global mx_list, my_list, running
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)  # 포트 안정화 대기
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    while running:
        try:
            line = ser.readline().decode("utf-8", errors='ignore').strip()
            if not line:
                continue

            parts = line.split(",")
            # 데이터 포맷에 맞게 인덱스 확인 (사용자 코드 기준 7, 8번이 mx, my)
            if len(parts) < 10:
                continue

            mx = float(parts[7])
            my = float(parts[8])

            mx_list.append(mx)
            my_list.append(my)

        except Exception:
            pass
    
    ser.close()
    print("Serial port closed.")

# ---- 캘리브레이션 계산 및 저장 함수 ----
# 기존 import에 numpy가 있어야 합니다.
import numpy as np

def calculate_and_save_calibration():
    if len(mx_list) < 50:
        print("\n[경고] 데이터가 너무 적어 캘리브레이션을 수행할 수 없습니다.")
        return

    print("\n--- Calculating Calibration Data (Hard Iron Only - Raw Scale) ---")
    
    mx_arr = np.array(mx_list)
    my_arr = np.array(my_list)
    
    # 데이터를 2D 배열로 변환
    data = np.column_stack([mx_arr, my_arr])

    # 1. Hard Iron (Offset) - 타원의 중심 찾기
    # 백분위수 사용하여 노이즈 제거
    min_x = np.percentile(mx_arr, 1)
    max_x = np.percentile(mx_arr, 99)
    min_y = np.percentile(my_arr, 1)
    max_y = np.percentile(my_arr, 99)
    
    mag_offset_x = (max_x + min_x) / 2.0
    mag_offset_y = (max_y + min_y) / 2.0
    
    # 2. Soft Iron (Scale) - 주석처리 (raw 데이터 그대로 사용)
    # # 중심을 원점으로 이동
    # data_centered = data - [mag_offset_x, mag_offset_y]
    # 
    # # PCA로 타원의 주축 찾기
    # # 공분산 행렬 계산
    # cov_matrix = np.cov(data_centered.T)
    # 
    # # 고유값과 고유벡터 계산 (주축 방향과 길이)
    # eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
    # 
    # # 고유값이 타원의 반경 제곱에 비례 (장축, 단축)
    # # 고유값의 제곱근이 반경
    # radii = np.sqrt(eigenvalues)
    # 
    # # 장축과 단축 구분
    # major_axis_idx = np.argmax(radii)
    # minor_axis_idx = np.argmin(radii)
    # 
    # major_radius = radii[major_axis_idx]
    # minor_radius = radii[minor_axis_idx]
    # 
    # if minor_radius == 0 or major_radius == 0:
    #     print("[에러] 데이터 범위가 너무 좁습니다. 센서를 더 크게 회전시켜주세요.")
    #     return
    # 
    # # 평균 반경 계산 (원으로 만들기 위한 목표 반경)
    # target_radius = (major_radius + minor_radius) / 2.0
    # 
    # # 주축 방향의 스케일 계산
    # scale_major = target_radius / major_radius  # 장축은 줄임
    # scale_minor = target_radius / minor_radius  # 단축은 늘림
    # 
    # # 주축 방향으로 변환하는 회전 행렬
    # # eigenvectors는 이미 정규화되어 있음
    # rotation_matrix = eigenvectors.T  # 주축 좌표계로 변환
    # 
    # # 스케일 행렬 (주축 좌표계에서)
    # scale_matrix = np.array([[scale_major, 0], [0, scale_minor]])
    # 
    # # 역변환: 주축 좌표계 -> 원래 좌표계
    # # 전체 변환 행렬: R^T * S * R
    # transform_matrix = rotation_matrix.T @ scale_matrix @ rotation_matrix
    # 
    # # X, Y 축에 대한 스케일 추출 (근사)
    # mag_scale_x = transform_matrix[0, 0]
    # mag_scale_y = transform_matrix[1, 1]
    # 
    # print(f"타원 분석:")
    # print(f"  장축 반경: {major_radius:.2f}")
    # print(f"  단축 반경: {minor_radius:.2f}")
    # print(f"  목표 반경: {target_radius:.2f}")
    # print(f"  장축 스케일: {scale_major:.4f}")
    # print(f"  단축 스케일: {scale_minor:.4f}")
    
    # Raw 데이터 그대로 사용 (스케일 = 1.0)
    mag_scale_x = 1.0
    mag_scale_y = 1.0

    calib_data = {
        "mag_offset_x": mag_offset_x,
        "mag_offset_y": mag_offset_y,
        "mag_scale_x": mag_scale_x,
        "mag_scale_y": mag_scale_y
    }

    filename = "mag_calibration.json"
    with open(filename, "w") as f:
        json.dump(calib_data, f, indent=4)

    print(f"Calibration saved to '{filename}'")
    print(f"Offsets -> X: {mag_offset_x:.2f}, Y: {mag_offset_y:.2f}")
    
    # 시각화 실행 (중심점 십자선 추가)
    visualize_calibration(mx_arr, my_arr, calib_data)

def visualize_calibration(mx, my, calib):
    plt.ioff()
    plt.figure(figsize=(8, 8))
    
    plt.scatter(mx, my, label='Raw Data', alpha=0.3, color='red', s=10)
    
    # Hard Iron offset만 적용 (Soft Iron 보정 없음)
    mx_cal = mx - calib['mag_offset_x']
    my_cal = my - calib['mag_offset_y']
    
    # # Soft Iron 보정 (주석처리)
    # if 'transform_matrix' in calib:
    #     # 변환 행렬 사용 (더 정확)
    #     data = np.column_stack([mx, my])
    #     data_centered = data - [calib['mag_offset_x'], calib['mag_offset_y']]
    #     transform = np.array(calib['transform_matrix'])
    #     data_calibrated = (transform @ data_centered.T).T
    #     mx_cal = data_calibrated[:, 0]
    #     my_cal = data_calibrated[:, 1]
    # else:
    #     # 기존 방식 (호환성)
    #     mx_cal = (mx - calib['mag_offset_x']) * calib['mag_scale_x']
    #     my_cal = (my - calib['mag_offset_y']) * calib['mag_scale_y']
    
    plt.scatter(mx_cal, my_cal, label='Calibrated Data (Hard Iron Only)', alpha=0.3, color='blue', s=10)
    
    # (0,0) 중심 십자선 그리기 (정확히 중심에 왔는지 확인용)
    plt.axhline(0, color='black', linewidth=1, linestyle='--')
    plt.axvline(0, color='black', linewidth=1, linestyle='--')

    plt.legend()
    plt.title("Magnetometer Calibration Result (Hard Iron Only - Raw Scale)")
    plt.axis('equal')
    plt.grid(True)
    plt.show()
# ---- 메인 ----
if __name__ == "__main__":
    try:
        port = find_port()
        # 데몬 스레드 대신 일반 스레드로 실행하고 플래그로 제어
        t = threading.Thread(target=serial_thread, args=(port,))
        t.start()

        plt.ion()  
        fig, ax = plt.subplots()
        sc = ax.scatter([], [])
        ax.set_title("Rotate sensor to calibrate (Press Ctrl+C to save)")
        ax.set_xlabel("mx")
        ax.set_ylabel("my")
        ax.axis('equal') # X, Y 비율을 1:1로 고정 (원형 확인 용이)

        print("Start rotating the sensor in all directions...")
        
        while True:
            if len(mx_list) > 0:
                # 성능을 위해 최신 500개 점만 표시하거나 전체 표시
                sc.set_offsets(list(zip(mx_list, my_list)))
                
                # 축 범위 동적 조정
                cur_min = min(min(mx_list), min(my_list))
                cur_max = max(max(mx_list), max(my_list))
                margin = (cur_max - cur_min) * 0.1
                
                ax.set_xlim(cur_min - margin, cur_max + margin)
                ax.set_ylim(cur_min - margin, cur_max + margin)
                
                plt.pause(0.05)
            else:
                plt.pause(0.1)

    except KeyboardInterrupt:
        print("\nStopping data collection...")
        running = False # 스레드 종료 신호
        t.join() # 스레드 종료 대기
        
        # 캘리브레이션 계산 및 저장 실행
        calculate_and_save_calibration()
    
    except Exception as e:
        print(f"Error: {e}")