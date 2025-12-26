import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import sys

def plot_latest_calibration():
    # 1. 가장 최신 로그 파일 찾기
    log_files = glob.glob("logs/calibration_*.csv")
    if not log_files:
        print("Error: No calibration logs found in logs/ directory.")
        return

    latest_file = max(log_files, key=os.path.getctime)
    print(f"Loading latest calibration log: {latest_file}")

    # 2. 데이터 읽기
    try:
        df = pd.read_csv(latest_file)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return

    if df.empty:
        print("Error: Log file is empty.")
        return

    # 첫 번째 타임스탬프를 기준으로 0초부터 시작하도록 변환
    start_time = df['timestamp'].iloc[0]
    df['relative_time'] = df['timestamp'] - start_time

    # 3. 데이터 분리
    speed_data = df[df['type'] == 'speed']
    adjust_up = df[(df['type'] == 'adjust') & (df['direction'] == '+')]
    adjust_down = df[(df['type'] == 'adjust') & (df['direction'] == '-')]

    # 4. 그래프 그리기
    plt.figure(figsize=(12, 6))
    
    # 속도 선 그래프 (인지에서 들어오는 정보)
    if not speed_data.empty:
        plt.plot(speed_data['relative_time'], speed_data['value'], label='Perception Speed (m/s)', color='gray', alpha=0.5, linestyle='--')
        plt.scatter(speed_data['relative_time'], speed_data['value'], color='black', s=1, alpha=0.3)

    # 증가(UP) 이벤트 표시 - 빨간색
    if not adjust_up.empty:
        # 이벤트 시점의 속도를 찾기 위해 병합(merge)이나 가장 가까운 값 찾기 대신 
        # y축 위치를 시각적으로 구분하기 위해 별도 표시하거나 속도 데이터와 오버레이
        # 여기서는 편의상 속도 그래프 위에 찍기 위해 속도 데이터와 시간 축을 맞춤
        plt.scatter(adjust_up['relative_time'], [max(speed_data['value']) * 1.05 if not speed_data.empty else 1.0] * len(adjust_up), 
                    color='red', label='Throttle UP (RB)', marker='^', s=100)

    # 감소(DOWN) 이벤트 표시 - 파란색
    if not adjust_down.empty:
        plt.scatter(adjust_down['relative_time'], [max(speed_data['value']) * 0.95 if not speed_data.empty else 0.5] * len(adjust_down), 
                    color='blue', label='Throttle DOWN (LB)', marker='v', s=100)

    plt.title(f"Speed Calibration Trace: {os.path.basename(latest_file)}")
    plt.xlabel("Time (seconds)")
    plt.ylabel("Speed (m/s)")
    plt.legend()
    plt.grid(True, which='both', linestyle='--', alpha=0.5)
    
    # 결과 저장
    output_path = "calibration_result.png"
    plt.savefig(output_path)
    print(f"Visualization saved to {output_path}")
    plt.show()

if __name__ == "__main__":
    plot_latest_calibration()
