import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
import argparse

# --- Configuration Constants (Analyzer와 동일하게 유지) ---
TOLERANCE = 0.05      # m/s

def visualize_log(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File not found: {file_path}")
        return

    print(f"[VISUALIZER] Loading log: {file_path}")
    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        print(f"[VISUALIZER] Error reading CSV: {e}")
        return

    # 필수 컬럼 체크 (battery -> battery_pct 대응)
    if 'battery' in df.columns and 'battery_pct' not in df.columns:
        df.rename(columns={'battery': 'battery_pct'}, inplace=True)

    # 시간 정규화
    start_time = df['timestamp'].iloc[0]
    df['relative_time'] = df['timestamp'] - start_time

    # [SPIKE SUPPRESSION]
    if len(df) > 2:
        neighbor_avg = (df['obs_value'].shift(1) + df['obs_value'].shift(-1)) / 2
        spike_mask = (df['obs_value'] > neighbor_avg + 1.2) & (df['obs_value'] > df['threshold'] + 0.5)
        df.loc[spike_mask, 'obs_value'] = neighbor_avg[spike_mask]

    speed_df = df[df['type'] == 'speed'].copy()
    adjust_df = df[df['type'] == 'auto_adjust'].copy()

    # 페이즈 식별 (analyzer와 동일 로직)
    df['param_str'] = df['inc'].astype(str) + "_" + df['dec'].astype(str)
    df['phase_change'] = df['param_str'] != df['param_str'].shift()
    df['phase_id'] = df['phase_change'].cumsum()
    phases = df.groupby('phase_id')

    phase_summary = []
    for pid, group in phases:
        p_speed = group[group['type'] == 'speed']
        if p_speed.empty: continue
        phase_summary.append({
            'start_time': group['relative_time'].iloc[0],
            'inc': group['inc'].iloc[0],
            'dec': group['dec'].iloc[0],
            'pid': pid
        })

    # 시각화
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 12), sharex=True)

    # [Top Plot: Speed]
    ax1.plot(speed_df['relative_time'], speed_df['cmd_speed'], label='Command Speed (m/s)', color='gray', alpha=0.3, linestyle='--')
    ax1.plot(speed_df['relative_time'], speed_df['threshold'], label='Threshold_V', color='red', alpha=0.7, linewidth=1)
    ax1.plot(speed_df['relative_time'], speed_df['obs_value'], label='Observed Speed (m/s)', color='green', linewidth=1.5)
    ax1.fill_between(speed_df['relative_time'], speed_df['threshold'] - TOLERANCE, speed_df['threshold'] + TOLERANCE, color='red', alpha=0.05)

    for p in phase_summary:
        ax1.axvline(x=p['start_time'], color='black', alpha=0.3, linestyle=':', linewidth=1)
        ax1.text(p['start_time'] + 0.1, ax1.get_ylim()[1] * 0.9, f"P{p['pid']}\n{p['inc']:.4f}\n{p['dec']:.4f}", fontsize=8, verticalalignment='top', alpha=0.7)

    stall_events = adjust_df[adjust_df['reason'] == 'stall_recovery']
    limit_events = adjust_df[adjust_df['reason'] == 'speed_limit']
    if not stall_events.empty:
        ax1.scatter(stall_events['relative_time'], [0.2]*len(stall_events), color='blue', label='Stall Recovery', marker='^', s=30)
    if not limit_events.empty:
        ax1.scatter(limit_events['relative_time'], limit_events['threshold'], color='darkred', label='Speed Limit', marker='v', s=30)

    ax1.set_title(f"Calibration Analysis: {os.path.basename(file_path)}")
    ax1.set_ylabel("Speed (m/s)")
    ax1.legend(loc='upper right', fontsize='x-small', ncol=2)
    ax1.grid(True, alpha=0.1)

    # [Middle Plot: Throttle]
    ax2.plot(speed_df['relative_time'], speed_df['value'], label='SPEED_5_PHYS (Throttle)', color='orange', linewidth=2)
    ax2_twin = ax2.twinx()
    ax2_twin.step(speed_df['relative_time'], speed_df['inc'], where='post', label='Inc', color='blue', alpha=0.3)
    ax2_twin.step(speed_df['relative_time'], speed_df['dec'].abs(), where='post', label='|Dec|', color='purple', alpha=0.3)
    ax2.set_ylabel("Throttle")
    ax2.legend(loc='upper left', fontsize='x-small')
    ax2_twin.legend(loc='upper right', fontsize='x-small')
    ax2.grid(True, alpha=0.1)

    # [Bottom Plot: Battery]
    battery_col = 'battery_pct' if 'battery_pct' in df.columns else 'battery'
    ax3.plot(df['relative_time'], df[battery_col], color='teal', label='Battery %' if 'pct' in battery_col else 'Battery Voltage (V)', linewidth=2)
    ax3.set_ylabel("Battery" + (" (%)" if "pct" in battery_col else " (V)"))
    ax3.grid(True, alpha=0.1)
    ax3.legend(loc='upper right', fontsize='x-small')

    plt.tight_layout()
    output_path = file_path.replace('.csv', '.png')
    plt.savefig(output_path)
    print(f"[VISUALIZER] Result saved to: {output_path}")
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", type=str, help="CSV log file path to visualize")
    args = parser.parse_args()

    if args.file:
        visualize_log(args.file)
    else:
        print("Usage: python -m jetracer.tools.calibrate_visualizer --file <csv_path>")
