#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XYMU("#XYMU=ax,ay,az,gx,gy,gz,temp,mx,my,mz#") 시리얼 데이터를 읽어
Madgwick 필터로 Roll/Pitch/Yaw(deg) 실시간 출력 (ROS 불필요).

특징
- 실시간 dt 사용 (샘플링 주기 자동 반영)
- 초기 정지 캘리브레이션(기본 8초): 자이로 바이어스만 추정 (가속도는 중력 포함 그대로 사용)
- 정지 구간 자동 감지 시 자이로 바이어스 지속 보정(EMA)
- 6DoF/9DoF 자동 전환(자기값 유효 시 tilt-comp 헤딩 약하게 혼합 가능)
- 축 매핑 옵션(raw / ned2enu), yaw 스무딩, CSV 로깅 옵션
"""

import argparse, re, math, time, sys, csv
from collections import deque
import numpy as np
import serial

# ----------------- 유틸 -----------------

XYMU_RE = re.compile(r"XYMU=([\-0-9.,]+)")

def parse_xymu(line: str):
    m = XYMU_RE.search(line)
    if not m:
        return None
    try:
        vals = [float(x) for x in m.group(1).split(",")]
        if len(vals) != 10:
            return None
        ax,ay,az, gx,gy,gz, temp, mx,my,mz = vals
        return ax,ay,az, gx,gy,gz, temp, mx,my,mz
    except Exception:
        return None

def clamp(x,a,b): return a if x<a else (b if x>b else x)
def wrap_deg(a): return (a+180.0)%360.0-180.0

def axis_map(x,y,z, mode):
    if mode == "raw":
        return x,y,z
    elif mode == "ned2enu":   # down→up 플립
        return x,y,-z
    return x,y,z

# ----------------- Madgwick -----------------

class Madgwick:
    def __init__(self, beta=0.3):
        self.beta = float(beta)
        self.q = np.array([1.0,0.0,0.0,0.0], dtype=float)  # [w,x,y,z]

    @staticmethod
    def _qmul(q,r):
        w1,x1,y1,z1 = q
        w2,x2,y2,z2 = r
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ], dtype=float)

    def update(self, gx,gy,gz, ax,ay,az, mx=None,my=None,mz=None, *, gyro_is_degps=True, dt=0.01):
        if gyro_is_degps:
            gx,gy,gz = np.radians([gx,gy,gz])

        q1,q2,q3,q4 = self.q
        a = np.array([ax,ay,az], dtype=float)
        an = np.linalg.norm(a)
        if an < 1e-6:  # 가속도 불량 시 건너뜀
            return self.q
        a /= an

        use_mag = (mx is not None and my is not None and mz is not None)

        if not use_mag:
            # 6DoF
            _2q1=2*q1; _2q2=2*q2; _2q3=2*q3; _2q4=2*q4
            _4q2=4*q2; _4q3=4*q3
            f1 = _2q2*q4 - _2q1*q3 - a[0]
            f2 = _2q1*q2 + _2q3*q4 - a[1]
            f3 = 1.0 - (_2q2*q2 + _2q3*q3) - a[2]
            J11=-_2q3; J12=_2q4; J13=-_2q1; J14=_2q2
            J21=_2q2; J22=_2q1; J23=_2q4; J24=_2q3
            J31=0.0  ; J32=-_4q2; J33=-_4q3; J34=0.0
            grad = np.array([
                J11*f1 + J21*f2 + J31*f3,
                J12*f1 + J22*f2 + J32*f3,
                J13*f1 + J23*f2 + J33*f3,
                J14*f1 + J24*f2 + J34*f3
            ], dtype=float)
        else:
            # 9DoF (간이)
            m = np.array([mx,my,mz], dtype=float)
            mn = np.linalg.norm(m)
            if mn>1e-6: m/=mn
            _2q1=2*q1; _2q2=2*q2; _2q3=2*q3; _2q4=2*q4
            hx = 2*m[0]*(0.5-q3*q3-q4*q4) + 2*m[1]*(q2*q3-q1*q4) + 2*m[2]*(q2*q4+q1*q3)
            hy = 2*m[0]*(q2*q3+q1*q4) + 2*m[1]*(0.5-q2*q2-q4*q4) + 2*m[2]*(q3*q4-q1*q2)
            bx = math.sqrt(hx*hx + hy*hy)
            bz = 2*m[0]*(q2*q4-q1*q3) + 2*m[1]*(q3*q4+q1*q2) + 2*m[2]*(0.5-q2*q2-q3*q3)
            f1 = _2q2*q4 - _2q1*q3 - a[0]
            f2 = _2q1*q2 + _2q3*q4 - a[1]
            f3 = 1.0 - 2*(q2*q2 + q3*q3) - a[2]
            f4 = 2*bx*(0.5-q3*q3-q4*q4) + 2*bz*(q2*q4 - q1*q3) - m[0]
            f5 = 2*bx*(q2*q3 - q1*q4) + 2*bz*(q1*q2 + q3*q4) - m[1]
            f6 = 2*bx*(q1*q3 + q2*q4) + 2*bz*(0.5 - q2*q2 - q3*q3) - m[2]
            grad = np.array([
              -2*q3*f1 + 2*q2*f2 - 2*bz*q3*f4 + (-2*bx*q4 + 2*bz*q2)*f5 + 2*bx*q3*f6,
               2*q4*f1 + 2*q1*f2 - 4*q2*f3 + ( 2*bz*q4)*f4 + (2*bx*q3 + 2*bz*q1)*f5 + (2*bx*q4 - 4*bz*q2)*f6,
              -2*q1*f1 + 2*q4*f2 - 4*q3*f3 + (-4*bx*q3 - 2*bz*q1)*f4 + (2*bx*q2 + 2*bz*q4)*f5 + (2*bx*q1 - 4*bz*q3)*f6,
               2*q2*f1 + 2*q3*f2 +               (-2*bx*q2)*f4 + (-2*bx*q1 + 2*bz*q3)*f5 + (2*bz*q2)*f6
            ], dtype=float)

        gnorm = np.linalg.norm(grad)
        if gnorm>1e-9:
            grad/=gnorm

        qdot = 0.5*self._qmul(self.q, np.array([0.0,gx,gy,gz])) - self.beta*grad
        self.q += qdot*float(dt)
        self.q /= np.linalg.norm(self.q)
        return self.q

def quat_to_euler_deg(q):
    w,x,y,z = q
    t0 = +2.0*(w*x + y*z); t1 = +1.0 - 2.0*(x*x + y*y)
    roll  = math.degrees(math.atan2(t0, t1))
    t2 = +2.0*(w*y - z*x); t2 = clamp(t2,-1.0,1.0)
    pitch = math.degrees(math.asin(t2))
    t3 = +2.0*(w*z + x*y); t4 = +1.0 - 2.0*(y*y + z*z)
    yaw   = math.degrees(math.atan2(t3, t4))
    return roll,pitch,yaw

# ----------------- 메인 -----------------

def main():
    ap = argparse.ArgumentParser(description="XYMU → RPY (no ROS)")
    ap.add_argument("--port", default="/dev/ttyACM0", help="serial port")
    ap.add_argument("--baud", type=int, default=115200, help="baudrate")
    ap.add_argument("--axis-mode", choices=["raw","ned2enu"], default="raw")
    ap.add_argument("--gyro-is-degps", action="store_true", default=True)
    ap.add_argument("--beta", type=float, default=0.3, help="Madgwick beta (정지 드리프트 억제↑)")
    ap.add_argument("--calib-sec", type=float, default=8.0, help="초기 정지 캘리브레이션 시간[s]")
    ap.add_argument("--mag-thresh", type=float, default=1e-3, help="|mx|+|my|+|mz| 임계")
    ap.add_argument("--tilt-comp-k", type=float, default=0.03, help="자기 헤딩 소프트 보정율(0=off)")
    ap.add_argument("--yaw-lpf", type=float, default=0.2, help="Yaw 지수평활 계수(0=off)")
    ap.add_argument("--print", dest="print_mode", choices=["rpy","yaw"], default="rpy")
    ap.add_argument("--log-csv", default="", help="CSV 경로 지정 시 로그 저장 (예: data.csv)")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    axis_mode = args.axis_mode

    # 초기 캘리브레이션: 자이로 바이어스(g_bias)와 g0(|acc| 평균)만 추정
    print(f"[INFO] Calibrating for {args.calib_sec:.1f}s… 센서 고정!")
    t0 = time.time()
    ncal=0
    gsum = np.zeros(3)
    amag_sum = 0.0
    while time.time()-t0 < args.calib_sec:
        line = ser.readline().decode(errors="ignore")
        p = parse_xymu(line)
        if not p: continue
        ax,ay,az, gx,gy,gz, *_ = p
        ax,ay,az = axis_map(ax,ay,az, axis_mode)
        gx,gy,gz = axis_map(gx,gy,gz, axis_mode)
        gsum += np.array([gx,gy,gz])
        amag_sum += math.sqrt(ax*ax + ay*ay + az*az)
        ncal += 1

    g_bias = gsum/ncal if ncal>0 else np.zeros(3)
    g0     = amag_sum/ncal if ncal>0 else 9.81
    print(f"[INFO] Calib done: g0≈{g0:.3f} m/s², g_bias={np.round(g_bias,3)}")

    filt = Madgwick(beta=args.beta)
    yaw_alpha = max(0.0, min(1.0, args.yaw_lpf))
    yaw_s = None

    # 지속 바이어스 보정용 파라미터
    alpha_bias = 1e-3           # 정지 시 자이로 바이어스 EMA
    gyro_static_thresh = 0.5    # deg/s (노름)
    acc_static_thresh = 0.15*9.81

    t_prev = time.time()
    last_rate_print = t_prev
    dt_hist = deque(maxlen=200)

    # CSV 로깅 준비
    csv_writer = None
    f_csv = None
    if args.log_csv:
        f_csv = open(args.log_csv, "w", newline="")
        csv_writer = csv.writer(f_csv)
        csv_writer.writerow(["t","roll_deg","pitch_deg","yaw_deg","ax","ay","az","gx","gy","gz","mx","my","mz"])
#python3 xymu_rpy.py --port /dev/ttyACM0 --baud 115200
    try:
        while True:
            line = ser.readline().decode(errors="ignore")
            p = parse_xymu(line)
            if not p: 
                continue
            ax,ay,az, gx,gy,gz, _, mx,my,mz = p

            # 축 매핑
            ax,ay,az = axis_map(ax,ay,az, axis_mode)
            gx,gy,gz = axis_map(gx,gy,gz, axis_mode)
            mx,my,mz = axis_map(mx,my,mz, axis_mode)

            # 자이로 바이어스 제거 (가속도는 빼지 않음!)
            gx -= g_bias[0]; gy -= g_bias[1]; gz -= g_bias[2]

            # dt
            t_now = time.time()
            dt = max(1e-4, t_now - t_prev)
            t_prev = t_now
            dt_hist.append(dt)

            # 정지 감지 → 자이로 바이어스 지속 보정
            gyro_norm = math.sqrt(gx*gx + gy*gy + gz*gz)         # 입력 단위(보통 deg/s)
            acc_norm  = math.sqrt(ax*ax + ay*ay + az*az)         # m/s²
            if gyro_norm < gyro_static_thresh and abs(acc_norm - g0) < acc_static_thresh:
                g_bias = (1.0 - alpha_bias)*g_bias + alpha_bias*np.array([gx,gy,gz])

            # mag 유효성 판단
            use_mag = (abs(mx)+abs(my)+abs(mz)) > float(args.mag_thresh)
            if not use_mag:
                mx=my=mz=None

            # 필터 업데이트
            q = filt.update(gx,gy,gz, ax,ay,az, mx,my,mz,
                            gyro_is_degps=args.gyro_is_degps, dt=dt)

            # RPY
            r,p,y = quat_to_euler_deg(q)

            # (옵션) mag 있을 때 tilt-comp heading으로 yaw를 살짝 끌어당김
            if use_mag and args.tilt_comp_k > 0.0:
                cr, sr = math.cos(math.radians(r)), math.sin(math.radians(r))
                cp, sp = math.cos(math.radians(p)), math.sin(math.radians(p))
                mxh = mx*cp + my*sr*sp + mz*cr*sp
                myh = my*cr - mz*sr
                hdg = math.degrees(math.atan2(-myh, mxh))  # ENU 가정(필요 시 부호 조정)
                err = wrap_deg(hdg - y)
                y = wrap_deg(y + args.tilt_comp_k * err)

            # Yaw 스무딩
            if yaw_alpha > 0.0:
                if yaw_s is None:
                    yaw_s = y
                else:
                    dy = wrap_deg(y - yaw_s)
                    yaw_s = wrap_deg(yaw_s + yaw_alpha * dy)
                y_out = yaw_s
            else:
                y_out = y

            # 출력
            if args.print_mode == "yaw":
                sys.stdout.write(f"\rYaw={y_out:7.2f}°     ")
            else:
                sys.stdout.write(f"\rRoll={r:7.2f}°, Pitch={p:7.2f}°, Yaw={y_out:7.2f}°     ")
            sys.stdout.flush()

            # CSV 로그
            if csv_writer:
                csv_writer.writerow([t_now, r, p, y_out, ax,ay,az, gx,gy,gz, (mx if mx is not None else 0.0),
                                     (my if my is not None else 0.0), (mz if mz is not None else 0.0)])

            # 레이트 표시
            if (t_now - last_rate_print) >= 2.0:
                if dt_hist:
                    avg_dt = sum(dt_hist)/len(dt_hist)
                    rate = 1.0/avg_dt if avg_dt>1e-6 else 0.0
                    sys.stdout.write(f"  |  rate≈{rate:5.1f} Hz   ")
                    sys.stdout.flush()
                last_rate_print = t_now

    except KeyboardInterrupt:
        print("\n[INFO] Exit.")
    finally:
        if f_csv: f_csv.close()
        ser.close()

if __name__ == "__main__":
    main()
