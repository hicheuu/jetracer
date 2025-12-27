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
    
    # 속도 선 그래프
    if not speed_data.empty:
        # 조이스틱으로 맞춘 물리적 스로틀 타겟 (Reference)
        # 스로틀 값(0.1~0.2)과 속도값(m/s)의 단위가 다르므로, 보조 축을 쓰거나 범례로만 표시
        # 여기서는 단순히 '기준선'으로서의 의미를 위해 스로틀 값을 같이 그림
        plt.plot(speed_data['relative_time'], speed_data['value'], label='Physical Throttle Target', color='orange', alpha=0.6, linestyle='-')
        
        # 인지된 실제 속도 (Observed) - 이게 메인 (Y축: m/s)
        plt.plot(speed_data['relative_time'], speed_data['obs_value'], label='BEV Observed Speed (m/s)', color='green', linewidth=2)

    # 증가(UP) 이벤트 표시 - 빨간색 (실제 속도 위에 표시)
    if not adjust_up.empty:
        y_pos = max(speed_data['obs_value']) * 1.1 if not speed_data.empty else 1.0
        plt.scatter(adjust_up['relative_time'], [y_pos] * len(adjust_up), 
                    color='red', label='Throttle UP (RB)', marker='^', s=100)

    # 감소(DOWN) 이벤트 표시 - 파란색 (실제 속도 근처에 표시)
    if not adjust_down.empty:
        y_pos = max(speed_data['obs_value']) * 0.9 if not speed_data.empty else 0.5
        plt.scatter(adjust_down['relative_time'], [y_pos] * len(adjust_down), 
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
