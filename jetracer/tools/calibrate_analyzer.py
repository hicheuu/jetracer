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
    required_cols = {'timestamp', 'type', 'obs_value', 'threshold', 'cmd_speed', 'value', 'inc', 'dec'}
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

    # 4. 데이터 분리 및 페이즈 식별
    speed_df = df[df['type'] == 'speed'].copy()
    adjust_df = df[df['type'] == 'auto_adjust'].copy()

    if speed_df.empty:
        print("[ANALYZER] Error: No speed data found in log.")
        return

    # 파라미터(inc, dec) 변화를 기준으로 페이즈 할당
    # inc 또는 dec가 이전 행과 다르면 페이즈 변경
    df['param_str'] = df['inc'].astype(str) + "_" + df['dec'].astype(str)
    df['phase_change'] = df['param_str'] != df['param_str'].shift()
    df['phase_id'] = df['phase_change'].cumsum()

    phases = df.groupby('phase_id')

    # 5. 분석 수행
    print("\n" + "="*80)
    print(f" [Calibration Analysis Report - Tuning Phases]")
    print(f" Log File: {os.path.basename(latest_file)}")
    print("="*80)
    print(f"{'Phase':<6} | {'Params (Inc/Dec)':<20} | {'Dur(s)':<8} | {'MAE':<8} | {'In-Tol':<8} | {'Loss(%)':<8}")
    print("-" * 80)

    phase_summary = []

    for pid, group in phases:
        # 해당 페이즈의 속도 데이터 필터링
        p_speed = group[group['type'] == 'speed']
        if p_speed.empty:
            continue
            
        p_active = p_speed[p_speed['cmd_speed'] >= MIN_CMD_SPEED]
        
        # 지속 시간 (송수신 간격 차분 합산)
        dt = p_speed['relative_time'].diff().fillna(0)
        p_duration = dt.sum()
        
        # 너무 짧은 페이즈(0.5초 미만)는 통계 노이즈로 간주하여 요약에서 제외 (선택 사항)
        if p_duration < 0.5:
            continue

        inc_val = group['inc'].iloc[0]
        dec_val = group['dec'].iloc[0]
        
        mae = 0.0
        within_range = 0.0
        if not p_active.empty:
            errors = p_active['obs_value'] - p_active['threshold']
            mae = errors.abs().mean()
            within_range = (errors.abs() <= TOLERANCE).mean() * 100
        
        # 패킷 손실
        p_lost = group['lost_packets'].sum() if 'lost_packets' in group.columns else 0
        p_total = len(group) + p_lost
        p_loss_rate = (p_lost / p_total * 100) if p_total > 0 else 0
        
        print(f"{pid:<6} | {inc_val:.4f}/{dec_val:.4f} | {p_duration:<8.2f} | {mae:<8.4f} | {within_range:<8.1f} | {p_loss_rate:<8.2f}")
        
        phase_summary.append({
            'pid': pid,
            'start_time': group['relative_time'].iloc[0],
            'end_time': group['relative_time'].iloc[-1],
            'params': f"Inc:{inc_val:.4f}, Dec:{dec_val:.4f}",
            'inc': inc_val,
            'dec': dec_val
        })

    print("="*80)
    
    # 전체 요약
    total_lost = df['lost_packets'].sum() if 'lost_packets' in df.columns else 0
    print(f" Total Packet Loss: {int(total_lost)} packets ({(total_lost/(len(df)+total_lost)*100):.2f}%)")
    print("="*80 + "\n")

    # 6. 시각화 개선
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

    # [Top Plot: Speed]
    ax1.plot(speed_df['relative_time'], speed_df['cmd_speed'], label='Command Speed (m/s)', color='gray', alpha=0.3, linestyle='--')
    ax1.plot(speed_df['relative_time'], speed_df['threshold'], label='Threshold_V', color='red', alpha=0.7, linewidth=1)
    ax1.plot(speed_df['relative_time'], speed_df['obs_value'], label='Observed Speed (m/s)', color='green', linewidth=1.5)
    
    # Tolerance band
    ax1.fill_between(speed_df['relative_time'], 
                     speed_df['threshold'] - TOLERANCE, 
                     speed_df['threshold'] + TOLERANCE, 
                     color='red', alpha=0.05)

    # 페이즈 구분선 및 텍스트
    for p in phase_summary:
        ax1.axvline(x=p['start_time'], color='black', alpha=0.3, linestyle=':', linewidth=1)
        # 페이즈 상단에 파라미터 정보 표시 (겹치지 않게 조절 필요할 수 있음)
        ax1.text(p['start_time'] + 0.1, ax1.get_ylim()[1] * 0.9, f"P{p['pid']}\n{p['inc']:.4f}\n{p['dec']:.4f}", 
                 fontsize=8, verticalalignment='top', alpha=0.7)

    # 이벤트 시각화
    stall_events = adjust_df[adjust_df['reason'] == 'stall_recovery']
    limit_events = adjust_df[adjust_df['reason'] == 'speed_limit']
    
    if not stall_events.empty:
        ax1.scatter(stall_events['relative_time'], [0.2]*len(stall_events), color='blue', label='Stall Recovery (UP)', marker='^', s=30, alpha=0.6)
    
    if not limit_events.empty:
        ax1.scatter(limit_events['relative_time'], limit_events['threshold'], color='darkred', label='Speed Limit (DOWN)', marker='v', s=30, alpha=0.6)

    ax1.set_title(f"Speed Calibration Analysis: {os.path.basename(latest_file)}")
    ax1.set_ylabel("Speed (m/s)")
    ax1.legend(loc='upper right', fontsize='x-small', ncol=2)
    ax1.grid(True, alpha=0.1)

    # [Bottom Plot: Physical Throttle & Params]
    ax2.plot(speed_df['relative_time'], speed_df['value'], label='SPEED_5_PHYS (Throttle)', color='orange', linewidth=2)
    
    # inc/dec 변화를 보조 축으로 그리기 (선택)
    ax2_twin = ax2.twinx()
    ax2_twin.step(speed_df['relative_time'], speed_df['inc'], where='post', label='Increment', color='blue', alpha=0.3)
    ax2_twin.step(speed_df['relative_time'], speed_df['dec'].abs(), where='post', label='|Decrement|', color='purple', alpha=0.3)
    ax2_twin.set_ylabel("Param Magnitude")
    
    ax2.set_title("Throttle progression & Parameter changes")
    ax2.set_xlabel("Time (seconds)")
    ax2.set_ylabel("Throttle Value")
    
    # 범례 합치기
    lines, labels = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2_twin.get_legend_handles_labels()
    ax2.legend(lines + lines2, labels + labels2, loc='upper right', fontsize='x-small')
    ax2.grid(True, alpha=0.1)

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
