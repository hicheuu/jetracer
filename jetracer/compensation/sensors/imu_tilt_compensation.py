#!/usr/bin/env python3
"""
Integrated IMU yaw estimator for JetRacer.

This is a single-run replacement for the old workflow:
  imu_reader.py -> imu_calibration.py -> imu_tilt_compensation.py

Behavior:
- Reads SparkFun 9DoF Razor IMU M0 serial stream (15 CSV fields).
- If a quaternion offset file exists, loads it. Otherwise calibrates on startup
  using the first N samples and saves the offset.
- Computes tilt-compensated quaternion yaw (yaw_q) which is the yaw value used
  for server telemetry.
- Optionally computes magnetometer heading and a 1D Kalman filtered heading
  for debugging display (does not affect yaw_q).
- Publishes yaw_q to shared memory at /dev/shm/jetracer_qyaw for other processes.

No I2C access; only serial + file I/O.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from typing import Dict, Optional, Tuple

import numpy as np
import serial
import serial.tools.list_ports


BAUD_RATE_DEFAULT = 115200
MODULE_DIR = os.path.dirname(__file__)
DEFAULT_MAG_CAL_PATH = os.path.join(MODULE_DIR, "mag_calibration.json")
DEFAULT_QUAT_OFFSET_PATH = os.path.join(MODULE_DIR, "quat_offset.json")
DEFAULT_YAW_SHM_PATH = "/dev/shm/jetracer_qyaw"


# ==========================================
# Quaternion utils (from firmware scripts)
# ==========================================
def quat_conj(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([w, -x, -y, -z], dtype=float)


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quat_to_euler_zyx(q: np.ndarray) -> Tuple[float, float, float]:
    w, x, y, z = q

    # roll
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))

    # pitch
    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    # yaw
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny, cosy))
    if yaw < 0:
        yaw += 360
    return roll, pitch, yaw


# ==========================================
# Kalman filter (from firmware scripts)
# ==========================================
class SimpleKalman1D:
    def __init__(self, R: float = 0.1, Q: float = 0.001):
        self.R = R
        self.Q = Q
        self.P = 1.0
        self.x = 0.0
        self.first_run = True

    def update(self, measurement: float) -> float:
        if self.first_run:
            self.x = measurement
            self.first_run = False
            return self.x

        self.P = self.P + self.Q

        delta = measurement - self.x
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360

        K = self.P / (self.P + self.R)
        self.x += K * delta
        self.P = (1 - K) * self.P

        if self.x < 0:
            self.x += 360
        elif self.x >= 360:
            self.x -= 360

        return self.x


# ==========================================
# Calibration loaders
# ==========================================
def load_mag_calibration(path: str) -> Dict[str, float]:
    """
    Supports both schemas:
    - Firmware: mag_offset_x/mag_offset_y/mag_scale_x/mag_scale_y
    - Teleop yaw_hold: mx_offset/my_offset/mx_scale/my_scale (with optional z keys)
    """
    defaults = {
        "mag_offset_x": 0.0,
        "mag_offset_y": 0.0,
        "mag_scale_x": 1.0,
        "mag_scale_y": 1.0,
    }
    if not os.path.exists(path):
        return defaults
    try:
        with open(path, "r", encoding="utf-8") as f:
            cal = json.load(f)
    except Exception:
        return defaults

    if "mag_offset_x" in cal and "mag_offset_y" in cal:
        return {
            "mag_offset_x": float(cal.get("mag_offset_x", 0.0)),
            "mag_offset_y": float(cal.get("mag_offset_y", 0.0)),
            "mag_scale_x": float(cal.get("mag_scale_x", 1.0)),
            "mag_scale_y": float(cal.get("mag_scale_y", 1.0)),
        }

    # fallback to yaw_hold schema
    return {
        "mag_offset_x": float(cal.get("mx_offset", 0.0)),
        "mag_offset_y": float(cal.get("my_offset", 0.0)),
        "mag_scale_x": float(cal.get("mx_scale", 1.0)),
        "mag_scale_y": float(cal.get("my_scale", 1.0)),
    }


def load_quat_offset(path: str) -> Optional[np.ndarray]:
    if not os.path.exists(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
        arr = np.array(raw, dtype=float)
        if arr.shape != (4,):
            return None
        n = np.linalg.norm(arr)
        if n <= 1e-9:
            return None
        return arr / n
    except Exception:
        return None


def save_quat_offset(path: str, q_offset: np.ndarray) -> None:
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump([float(x) for x in q_offset], f, indent=2)
    except Exception:
        pass


# ==========================================
# IMU stream utils
# ==========================================
def find_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No IMU detected")
    for port in ports:
        if "ACM" in port.device or "USB" in port.device or "usbmodem" in port.device:
            return port.device
    return ports[0].device


def parse_imu_data(line: str) -> Optional[Dict[str, float]]:
    parts = line.split(",")
    if len(parts) < 15:
        return None
    try:
        return {
            "mx": float(parts[7]),
            "my": float(parts[8]),
            "qw": float(parts[10]),
            "qx": float(parts[11]),
            "qy": float(parts[12]),
            "qz": float(parts[13]),
        }
    except Exception:
        return None


def simple_heading(mx: float, my: float, cal: Dict[str, float]) -> float:
    mx = (mx - cal["mag_offset_x"]) * cal["mag_scale_x"]
    my = (my - cal["mag_offset_y"]) * cal["mag_scale_y"]
    heading = math.degrees(math.atan2(my, mx))
    if heading < 0:
        heading += 360
    return heading


# ==========================================
# Main estimator
# ==========================================
class IMUTiltCompensator:
    def __init__(
        self,
        mag_cal: Dict[str, float],
        quat_offset: Optional[np.ndarray],
        quat_calib_samples: int,
        quat_offset_path: str,
        publish_shm_path: Optional[str],
        verbose: bool,
    ):
        self.mag_cal = mag_cal
        self.q_offset = quat_offset
        self.quat_calib_samples = max(1, int(quat_calib_samples))
        self.quat_offset_path = quat_offset_path
        self.publish_shm_path = publish_shm_path
        self.verbose = verbose

        self._quat_buf: list[np.ndarray] = []
        self._kf = SimpleKalman1D(R=0.2, Q=0.005)

        if self.q_offset is None and self.verbose:
            print(f"[imu] calibrating quaternion offset from first {self.quat_calib_samples} samples")

    def _maybe_calibrate_quat(self, q_raw: np.ndarray) -> None:
        if self.q_offset is not None:
            return
        self._quat_buf.append(q_raw)
        if len(self._quat_buf) >= self.quat_calib_samples:
            q_calib = np.mean(np.stack(self._quat_buf, axis=0), axis=0)
            q_calib /= np.linalg.norm(q_calib)
            self.q_offset = quat_conj(q_calib)
            save_quat_offset(self.quat_offset_path, self.q_offset)
            self._quat_buf.clear()
            if self.verbose:
                print(f"\n[imu] q_offset calibrated and saved to {self.quat_offset_path}")

    def update(self, d: Dict[str, float]) -> Tuple[float, float, float]:
        q_raw = np.array([d["qw"], d["qx"], d["qy"], d["qz"]], dtype=float)
        q_raw = q_raw / np.linalg.norm(q_raw)

        self._maybe_calibrate_quat(q_raw)
        if self.q_offset is None:
            # still calibrating; fall back to raw yaw
            q_corr = q_raw
        else:
            q_corr = quat_mul(self.q_offset, q_raw)
            q_corr = q_corr / np.linalg.norm(q_corr)

        _, _, yaw_q = quat_to_euler_zyx(q_corr)
        h_mag = simple_heading(d["mx"], d["my"], self.mag_cal)
        h_filtered = self._kf.update(h_mag)

        if self.publish_shm_path:
            try:
                with open(self.publish_shm_path, "w") as f:
                    f.write(f"{yaw_q:.3f}")
            except Exception:
                pass

        return yaw_q, h_mag, h_filtered


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="IMU quaternion yaw (tilt compensated) publisher")
    p.add_argument("--port", default=None, help="serial port (auto-detect if omitted)")
    p.add_argument("--baud", type=int, default=BAUD_RATE_DEFAULT, help="serial baud rate")
    p.add_argument("--mag-cal-path", default=DEFAULT_MAG_CAL_PATH, help="mag calibration json path")
    p.add_argument("--quat-offset-path", default=DEFAULT_QUAT_OFFSET_PATH, help="quaternion offset json path")
    p.add_argument(
        "--recalibrate-quat",
        action="store_true",
        help="ignore existing quat offset and recalibrate on startup",
    )
    p.add_argument(
        "--quat-calib-samples",
        type=int,
        default=500,
        help="number of samples for quaternion offset calibration (default: 500)",
    )
    p.add_argument(
        "--yaw-shm-path",
        default=DEFAULT_YAW_SHM_PATH,
        help="shared memory path to publish yaw_q (set empty to disable)",
    )
    p.add_argument("--verbose", action="store_true", help="print streaming yaw values")
    return p


def main():
    args = build_parser().parse_args()

    port = args.port or find_port()
    mag_cal = load_mag_calibration(args.mag_cal_path)

    quat_offset = None if args.recalibrate_quat else load_quat_offset(args.quat_offset_path)
    publish_shm_path = args.yaw_shm_path or None

    if args.verbose:
        print("=" * 60)
        print("JetRacer IMU Tilt Compensation (integrated)")
        print("=" * 60)
        print(f"[imu] port={port} baud={args.baud}")
        print(f"[imu] mag_cal={args.mag_cal_path}")
        print(f"[imu] quat_offset={args.quat_offset_path} loaded={quat_offset is not None}")
        if publish_shm_path:
            print(f"[imu] publishing yaw_q -> {publish_shm_path}")

    ser = serial.Serial(port, args.baud, timeout=1)
    time.sleep(0.5)

    estimator = IMUTiltCompensator(
        mag_cal=mag_cal,
        quat_offset=quat_offset,
        quat_calib_samples=args.quat_calib_samples,
        quat_offset_path=args.quat_offset_path,
        publish_shm_path=publish_shm_path,
        verbose=args.verbose,
    )

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            d = parse_imu_data(line)
            if not d:
                continue
            yaw_q, h_mag, h_filtered = estimator.update(d)

            if args.verbose:
                print(
                    f"\rQ-Yaw:{yaw_q:6.1f}° | Mag:{h_mag:6.1f}° | Filt:{h_filtered:6.1f}°",
                    end="",
                    flush=True,
                )
    except KeyboardInterrupt:
        if args.verbose:
            print("\n[imu] interrupted")
    finally:
        ser.close()


if __name__ == "__main__":
    main()

