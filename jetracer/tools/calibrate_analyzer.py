import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import sys
import numpy as np

# --- Configuration Constants ---
MIN_CMD_SPEED = 0.5  # 분석 대상이 되는 최소 명령 속도
TOLERANCE = 0.05      # m/s (Target Speed Control 평가 시 허용 오차)

def analyze_latest_calibration():
    # 1. 가장 최신 로그 파일 찾기
    log_files = glob.glob("logs/calibration_*.csv")
    if not log_files:
        print("Error: No calibration logs found in logs/ directory.")
        return

    latest_file = max(log_files, key=os.path.getctime)
    print(f"[ANALYZER] Loading log: {latest_file}")

    # 2. 데이터 읽기
    try:
        df = pd.read_csv(latest_file)
    except Exception as e:
        print(f"[ANALYZER] Error reading CSV: {e}")
        return

    # 3. 방어적 코드: 필수 컬럼 체크
    required_cols = {'timestamp', 'type', 'obs_value', 'threshold', 'cmd_speed', 'value'}
    missing = required_cols - set(df.columns)
    if missing:
        print(f"[ANALYZER] Error: Missing required columns in CSV: {missing}")
        return

    if df.empty:
        print("[ANALYZER] Error: Log file is empty.")
        return

    # 첫 번째 타임스탬프를 기준으로 0초부터 시작하도록 변환
    start_time = df['timestamp'].iloc[0]
    df['relative_time'] = df['timestamp'] - start_time

    # 4. 데이터 분리
    speed_df = df[df['type'] == 'speed'].copy()
    adjust_df = df[df['type'] == 'auto_adjust'].copy()

    if speed_df.empty:
        print("[ANALYZER] Error: No speed data found in log.")
        return

    # 5. 분석 수행
    print("\n" + "="*50)
    print(f" [Calibration Analysis Report]")
    print(f" Log File: {os.path.basename(latest_file)}")
    print("="*50)

    # (1) Threshold 준수율 및 오차 분석 (다각도 지표)
    active_speed = speed_df[speed_df['cmd_speed'] >= MIN_CMD_SPEED]
    if not active_speed.empty:
        # 오차 계산 (Observed - Threshold)
        errors = active_speed['obs_value'] - active_speed['threshold']
        mae = errors.abs().mean()
        
        # 준수율 정의 개선
        overshoot_rate = (errors > TOLERANCE).mean() * 100
        undershoot_rate = (errors < -TOLERANCE).mean() * 100
        within_range = (errors.abs() <= TOLERANCE).mean() * 100
        
        print(f"1. Target Speed Control (Tolerance ±{TOLERANCE} m/s)")
        print(f"   - Mean Absolute Error: {mae:.4f} m/s")
        print(f"   - In Range (±Tol): {within_range:.1f}%")
        print(f"   - Overshoot (>Tol): {overshoot_rate:.1f}%")
        print(f"   - Undershoot (<Tol): {undershoot_rate:.1f}%")
        print(f"   - Max Overshoot: {errors.max():.4f} m/s")
    
    # (2) 보정 이벤트 및 패킷 손실
    stall_events = adjust_df[adjust_df['reason'] == 'stall_recovery']
    limit_events = adjust_df[adjust_df['reason'] == 'speed_limit']
    total_lost = df['lost_packets'].sum() if 'lost_packets' in df.columns else 0
    total_packets = len(df) + total_lost
    loss_rate = (total_lost / total_packets * 100) if total_packets > 0 else 0
    
    print(f"\n2. Network & Events")
    print(f"   - Total Adjustments: {len(adjust_df)}")
    print(f"   - Stall Recovery (UP): {len(stall_events)} times")
    print(f"   - Speed Limit (DOWN): {len(limit_events)} times")
    print(f"   - Total Packet Loss: {int(total_lost)} packets ({loss_rate:.2f}%)")

    # (3) Stall Recovery 효율성 (시간 기반 계산)
    stalls = active_speed[active_speed['obs_value'] <= 0.5]
    if not stalls.empty:
        # 시간 기반 계산: 마지막 스톨 - 첫 스톨 (단순 합산이 아닌 전체 구간 파악 시)
        # 여기서는 송수신 간격 차분을 합산하여 실제 "활동 시간" 중 스톨 시간을 계산
        dt = active_speed['relative_time'].diff().fillna(0)
        stall_mask = active_speed['obs_value'] <= 0.5
        total_stall_time = dt[stall_mask].sum()
        
        print(f"\n3. Stall Analysis (Obs < 0.5 m/s)")
        print(f"   - Total Stalled Duration: {total_stall_time:.2f} s")
        if not stall_events.empty:
            print(f"   - Recovery Efficiency: {len(stall_events)/total_stall_time:.2f} adjustments/sec during stall")
    print("="*50 + "\n")

    # 6. 시각화 개선
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

    # [Top Plot: Speed]
    ax1.plot(speed_df['relative_time'], speed_df['cmd_speed'], label='Command Speed (m/s)', color='gray', alpha=0.4, linestyle='--')
    ax1.plot(speed_df['relative_time'], speed_df['threshold'], label='Threshold_V', color='red', alpha=0.8, linewidth=1)
    ax1.plot(speed_df['relative_time'], speed_df['obs_value'], label='Observed Speed (m/s)', color='green', linewidth=2)
    
    # Tolerance band
    ax1.fill_between(speed_df['relative_time'], 
                     speed_df['threshold'] - TOLERANCE, 
                     speed_df['threshold'] + TOLERANCE, 
                     color='red', alpha=0.1, label=f'Tolerance ±{TOLERANCE}')

    # 이벤트 시각화 (Y좌표를 threshold에 맞춰 직관성 개선)
    if not stall_events.empty:
        ax1.scatter(stall_events['relative_time'], [0.2]*len(stall_events), color='blue', label='Stall Recovery (UP)', marker='^', s=40)
        # Stall 지점에 vertical line 추가 (선택)
        for t in stall_events['relative_time']:
            ax1.axvline(x=t, color='blue', alpha=0.1, linewidth=0.5)
    
    if not limit_events.empty:
        ax1.scatter(limit_events['relative_time'], limit_events['threshold'], color='darkred', label='Speed Limit (DOWN)', marker='v', s=40)

    ax1.set_title(f"Speed Calibration Analysis: {os.path.basename(latest_file)}")
    ax1.set_ylabel("Speed (m/s)")
    ax1.legend(loc='upper right', fontsize='small', ncol=2)
    ax1.grid(True, alpha=0.2)

    # [Bottom Plot: Physical Throttle]
    ax2.plot(speed_df['relative_time'], speed_df['value'], label='Physical Throttle (SPEED_5_PHYS)', color='orange', linewidth=2)
    ax2.set_title("Throttle Calibration Progression")
    ax2.set_xlabel("Time (seconds)")
    ax2.set_ylabel("Value")
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.2)

    plt.tight_layout()
    
    # 결과 저장
    output_path = "calibration_result.png"
    plt.savefig(output_path)
    print(f"[ANALYZER] Visualization saved to: {os.path.abspath(output_path)}")
    
    try:
        plt.show()
    except:
        pass

if __name__ == "__main__":
    analyze_latest_calibration()
