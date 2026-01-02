import pandas as pd
import glob
import os
import sys
import numpy as np

# --- Configuration Constants ---
MIN_CMD_SPEED = 0.5  # 분석 대상이 되는 최소 명령 속도
TOLERANCE = 0.05      # m/s (Target Speed Control 평가 시 허용 오차)

def analyze_latest_calibration():
    # 1. 가장 최신 로그 파일 찾기 (재귀적으로 하위 디렉토리까지 탐색)
    pattern = "logs/**/calibration_*.csv"
    log_files = glob.glob(pattern, recursive=True)
    if not log_files:
        print(f"Error: No calibration logs found in logs/ directory.")
        return

    latest_file = max(log_files, key=os.path.getctime)
    print(f"[ANALYZER] Loading log: {latest_file}")

    # 2. 데이터 읽기
    try:
        df = pd.read_csv(latest_file)
        if df.empty:
            print(f"[ANALYZER] Warning: Log file '{latest_file}' is empty. Skipping analysis.")
            return
    except pd.errors.EmptyDataError:
        print(f"[ANALYZER] Warning: Log file '{latest_file}' has no data. Skipping analysis.")
        return
    except Exception as e:
        print(f"[ANALYZER] Error reading CSV: {e}")
        return

    # 3. 필수 컬럼 체크 (battery_pct 대응)
    if 'battery_pct' not in df.columns and 'battery' in df.columns:
        df.rename(columns={'battery': 'battery_pct'}, inplace=True)
        
    required_cols = {'timestamp', 'type', 'obs_value', 'threshold', 'cmd_speed', 'value', 'inc', 'dec', 'battery_pct'}
    missing = required_cols - set(df.columns)
    if missing:
        print(f"[ANALYZER] Error: Missing required columns in CSV: {missing}")
        return

    if df.empty:
        print("[ANALYZER] Error: Log file is empty. Skipping analysis.")
        return

    # 시간 정규화
    start_time = df['timestamp'].iloc[0]
    df['relative_time'] = df['timestamp'] - start_time

    # [SPIKE SUPPRESSION] 
    if len(df) > 2:
        neighbor_avg = (df['obs_value'].shift(1) + df['obs_value'].shift(-1)) / 2
        spike_mask = (df['obs_value'] > neighbor_avg + 1.2) & (df['obs_value'] > df['threshold'] + 0.5)
        df.loc[spike_mask, 'obs_value'] = neighbor_avg[spike_mask]

    # 4. 페이즈 분석
    df['param_str'] = df['inc'].astype(str) + "_" + df['dec'].astype(str)
    df['phase_change'] = df['param_str'] != df['param_str'].shift()
    df['phase_id'] = df['phase_change'].cumsum()
    phases = df.groupby('phase_id')

    print("\n" + "="*80)
    print(f" [Calibration Analysis Report - Tuning Phases]")
    print(f" Log File: {os.path.basename(latest_file)}")
    print("="*80)
    print(f"{'Phase':<6} | {'Params (Inc/Dec)':<20} | {'Dur(s)':<8} | {'MAE':<8} | {'Stable':<8} | {'Loss(%)':<8}")
    print("-" * 80)

    phase_summary = []
    for pid, group in phases:
        p_speed = group[group['type'] == 'speed']
        if p_speed.empty: continue
            
        p_active = p_speed[p_speed['cmd_speed'] >= MIN_CMD_SPEED]
        dt = p_speed['relative_time'].diff().fillna(0)
        p_duration = dt.sum()
        
        if p_duration < 0.5: continue

        inc_val = group['inc'].iloc[0]
        dec_val = group['dec'].iloc[0]
        
        mae = 0.0
        stable_pct = 0.0
        if not p_active.empty:
            errors = p_active['obs_value'] - p_active['threshold']
            mae = errors.abs().mean()
            stable_pct = (p_active['obs_value'] <= p_active['threshold'] + TOLERANCE).mean() * 100
        
        p_lost = group['lost_packets'].sum() if 'lost_packets' in group.columns else 0
        p_total = len(group) + p_lost
        p_loss_rate = (p_lost / p_total * 100) if p_total > 0 else 0
        
        print(f"{pid:<6} | {inc_val:.4f}/{dec_val:.4f} | {p_duration:<8.2f} | {mae:<8.4f} | {stable_pct:<8.1f} | {p_loss_rate:<8.2f}")
        
        phase_summary.append({'stable': stable_pct, 'duration': p_duration})

    print("="*80)
    
    # 배터리 정보 소급 적용: 기존 v 방식 파일명을 위해 전압 우선 시도
    battery_col = 'battery_v' if 'battery_v' in df.columns else 'battery_pct'
    valid_b = df[battery_col].dropna()
    valid_b = valid_b[valid_b > 0]
    b_start, b_end = (valid_b.iloc[0], valid_b.iloc[-1]) if not valid_b.empty else (0.0, 0.0)
    
    total_duration = df['relative_time'].iloc[-1]
    
    # 리포트에는 전압/SOC 모두 표시 (있다면)
    b_status_str = f" Battery: {b_start:.2f} -> {b_end:.2f}"
    if 'battery_pct' in df.columns:
        valid_soc = df['battery_pct'].dropna()
        valid_soc = valid_soc[valid_soc > 0]
        if not valid_soc.empty:
            b_status_str += f" ({valid_soc.iloc[0]:.0f}% -> {valid_soc.iloc[-1]:.0f}%)"
    
    print(b_status_str)
    print("="*80 + "\n")

    # 5. 파일명 변경 (기존 v 방식 복구 + 평균 스로틀 추가)
    try:
        # 마지막 행의 inc/dec 값을 10000배 하여 정수로 표시 (예: 0.001 -> in10)
        final_inc = int(df['inc'].iloc[-1] * 10000)
        final_dec = int(df['dec'].iloc[-1] * 10000)
        
        # 주행 중 평균 스로틀(value 컬럼) 계산 (0.352 -> 352 형태로 저장)
        avg_thr = int(df['value'].mean() * 1000) if not df['value'].empty else 0
        
        # 마지막 페이즈의 Stable 지표 가져오기
        last_stable = int(phase_summary[-1]['stable']) if phase_summary else 0
        
        dir_name = os.path.dirname(latest_file)
        # 중요: 기존에 요청하셨던 v8.4-8.2 형식을 위해 배터리 값 사용 (b_start, b_end)
        # throttle 정보(thr{avg_thr})를 추가
        new_name = os.path.join(dir_name, f"calib_in{final_inc}_de{final_dec}_dur{int(total_duration)}_stable{last_stable}_thr{avg_thr}_v{b_start:.1f}-{b_end:.1f}.csv")
        
        if os.path.exists(new_name):
            # 중복 방지: 동일 설정으로 여러 번 주행 시 시각 추가
            suffix = datetime.now().strftime("_%H%M%S")
            new_name = new_name.replace(".csv", f"{suffix}.csv")
            
        os.rename(latest_file, new_name)
        print(f"[ANALYZER] Log file renamed: {new_name}")
        print(f"[ANALYZER] Result Summary: Inc={final_inc}, Dec={final_dec}, Stable={last_stable}%, AvgThr={avg_thr/1000:.3f}")
    except Exception as e:
        print(f"[ANALYZER] Error renaming file: {e}")

if __name__ == "__main__":
    analyze_latest_calibration()
