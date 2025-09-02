#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XYMU 원시 9축(raw) → 축재정의/부호/프레임 선택 → tilt-compensated YPR

입력 포맷 예)
#XYMU=ax,ay,az,gx,gy,gz,mx,my,mz,(temp)#  ← 숫자 9~10개

증상: yaw가 요동하거나 요 회전에 반응이 없을 때
원인: 보드 장착 방향에 따른 축 순서/부호/프레임 불일치가 거의 100%

해결: 아래 옵션으로 즉석 교정
  --acc-idx "0,1,2"     가속도 축 인덱스 재배열(원본: ax=0 ay=1 az=2)
  --acc-sign "1,1,1"    가속도 축 부호
  --mag-idx "0,1,2"     자력계 축 인덱스 재배열(원본: mx=0 my=1 mz=2)
  --mag-sign "1,1,1"    자력계 축 부호
  --frame android|enu|ned   프레임 정의( yaw 부호/정의에 영향 )
  --decl 8.0             자기 편차(도) +동경(동쪽이 +)
  --mbias "bx,by,bz"     하드아이언 바이어스
  --mscale "sx,sy,sz"    소프트아이언 스케일(축별 간단버전)
  --rad                   라디안 출력
  --stdin                 표준입력에서 줄 읽기(로그로 테스트)
"""

import sys, re, math, argparse
from datetime import datetime
try:
    import serial
except ImportError:
    serial = None

XYMU_RE = re.compile(r'#\s*XYMU\s*=\s*([^#]+?)\s*#')

def parse_xymu_payload(line):
    m = XYMU_RE.search(line)
    if not m: return None
    parts = [p.strip() for p in m.group(1).split(',') if p.strip()]
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        return None
    if len(vals) < 9:  # 최소 A(3)+G(3)+M(3)
        return None
    return vals

def remap_sign(v3, idx, sgn):
    # v3: (x,y,z), idx: (i,j,k), sgn: (sx,sy,sz)
    return (sgn[0]*v3[idx[0]], sgn[1]*v3[idx[1]], sgn[2]*v3[idx[2]])

def apply_mag_cal(mx,my,mz, bias, scale):
    return ((mx - bias[0]) * scale[0],
            (my - bias[1]) * scale[1],
            (mz - bias[2]) * scale[2])

def ypr_from_acc_mag(ax, ay, az, mx, my, mz, frame="android", decl_deg=0.0):
    """
    표준 경사보정:
      roll  = atan2(ay, az)
      pitch = atan2(-ax, sqrt(ay^2 + az^2))
      mx2 = mx*cos(pitch) + mz*sin(pitch)
      my2 = mx*sin(roll)*sin(pitch) + my*cos(roll) - mz*sin(roll)*cos(pitch)
    yaw = atan2( ? , ? )  ← 프레임 정의별로 부호/순서가 달라짐
    """
    # 방어
    if (ax,ay,az) == (0.0,0.0,0.0): return None

    roll  = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))

    # tilt compensation
    mx2 = mx*math.cos(pitch) + mz*math.sin(pitch)
    my2 = (mx*math.sin(roll)*math.sin(pitch) +
           my*math.cos(roll) -
           mz*math.sin(roll)*math.cos(pitch))

    # 프레임별 yaw 정의
    if frame == "android":
        # 안드로이드/항공기 좌표와 동일하게 동작하는 경우가 많음
        # (앞이 +X, 오른쪽 +Y, 위 +Z 가정일 때 자주 쓰는 형태)
        yaw = math.atan2(-my2, mx2)
    elif frame == "enu":
        # ENU(East,North,Up): heading = atan2(East, North) = atan2(mx2, my2)
        yaw = math.atan2(mx2, my2)
    elif frame == "ned":
        # NED(North,East,Down): heading = atan2(East, North) = atan2(my2, mx2)
        yaw = math.atan2(my2, mx2)
    else:
        yaw = math.atan2(-my2, mx2)  # 기본

    # 자기 편차(동쪽이 +)
    yaw += math.radians(decl_deg)

    # 정규화 (-pi, pi]
    if yaw > math.pi: yaw -= 2*math.pi
    if yaw <= -math.pi: yaw += 2*math.pi

    return yaw, pitch, roll, mx2, my2

def deg(x): return x*180.0/math.pi

def read_from_serial(dev, baud=115200, timeout=1.0):
    if serial is None:
        raise RuntimeError("pyserial 미설치: `pip install pyserial`")
    with serial.Serial(dev, baudrate=baud, timeout=timeout) as ser:
        while True:
            raw = ser.readline()
            if not raw: continue
            yield raw.decode("utf-8", errors="ignore").rstrip("\r\n")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/serial/by-id/usb-SparkFun_SFE_9DOF-D21_F22F6F2A50533357322E3120FF090637-if00")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--stdin", action="store_true")
    ap.add_argument("--rad", action="store_true")
    ap.add_argument("--frame", choices=["android","enu","ned"], default="android")
    ap.add_argument("--decl", type=float, default=0.0, help="자기 편차(deg, 동쪽 +)")
    ap.add_argument("--acc-idx", default="0,1,2", help="예: '0,2,1'")
    ap.add_argument("--acc-sign", default="1,1,1", help="예: '1,-1,1'")
    ap.add_argument("--mag-idx", default="0,1,2", help="예: '1,0,2'")
    ap.add_argument("--mag-sign", default="1,1,1", help="예: '-1,1,1'")
    ap.add_argument("--mbias", default="0,0,0", help="예: '0.055,-0.6,0.0'")
    ap.add_argument("--mscale", default="1,1,1", help="예: '38.73,0.68,0.66' (주의: 과대스케일이면 비정상)")
    args = ap.parse_args()

    acc_idx  = tuple(int(x) for x in args.acc_idx.split(","))
    acc_sign = tuple(float(x) for x in args.acc_sign.split(","))
    mag_idx  = tuple(int(x) for x in args.mag_idx.split(","))
    mag_sign = tuple(float(x) for x in args.mag_sign.split(","))

    mbias  = tuple(float(x) for x in args.mbias.split(","))
    mscale = tuple(float(x) for x in args.mscale.split(","))

    src = sys.stdin if args.stdin else read_from_serial(args.dev, args.baud)

    while True:
        try:
            line = next(src)
        except StopIteration:
            break
        except Exception:
            continue

        # 한 줄에 여러 블록이 이어붙어 있어도 모두 잡아냄
        for m in XYMU_RE.finditer(line):
            vals = parse_xymu_payload(m.group(0))
            if not vals: continue

            ax, ay, az = vals[0], vals[1], vals[2]
            # gx, gy, gz = vals[3], vals[4], vals[5]  # 현재 YPR 미사용
            mx, my, mz = vals[6], vals[7], vals[8]

            # 축 재매핑/부호 적용
            ax, ay, az = remap_sign((ax,ay,az), acc_idx, acc_sign)
            mx, my, mz = remap_sign((mx,my,mz), mag_idx, mag_sign)

            # 자력계 보정
            mx, my, mz = apply_mag_cal(mx,my,mz, mbias, mscale)

            ypr = ypr_from_acc_mag(ax, ay, az, mx, my, mz, frame=args.frame, decl_deg=args.decl)
            if ypr is None: continue
            yaw, pitch, roll, mx2, my2 = ypr

            unit = "rad" if args.rad else "deg"
            if not args.rad:
                yaw, pitch, roll = deg(yaw), deg(pitch), deg(roll)

            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(
                f"[{ts}] YPR: yaw={yaw:+7.3f} {unit}, pitch={pitch:+7.3f} {unit}, roll={roll:+7.3f} {unit} | "
                f"ACC=({ax:+.2f},{ay:+.2f},{az:+.2f}) MAG=({mx:+.2f},{my:+.2f},{mz:+.2f}) mx2={mx2:+.3f} my2={my2:+.3f} frame={args.frame}"
            )

if __name__ == "__main__":
    main()
