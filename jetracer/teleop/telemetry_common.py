#!/usr/bin/env python3
"""
Common helpers for JetRacer telemetry.

- Battery voltage is read from /dev/shm/jetracer_voltage
- Heading delta (Δψ) is read from /dev/shm/jetracer_heading_delta
  (written by imu_yaw_delta_writer @ 30Hz)

No I2C or serial access here.
"""

from __future__ import annotations

import os
import re
import struct
from typing import Optional, Tuple

# =========================
# Battery SOC helpers
# =========================

SOC_TABLE = [
    (4.20, 100),
    (4.00, 85),
    (3.85, 60),
    (3.70, 40),
    (3.50, 20),
    (3.30, 10),
    (3.00, 0),
]


def soc_from_voltage(pack_v: float, cells: int = 2) -> int:
    vpc = pack_v / max(1, cells)
    if vpc >= 4.20:
        return 100
    if vpc <= 3.00:
        return 0
    for (vh, sh), (vl, sl) in zip(SOC_TABLE, SOC_TABLE[1:]):
        if vl <= vpc <= vh:
            t = (vpc - vl) / (vh - vl)
            return int(sl + t * (sh - sl))
    return 0


def read_float_file(path: str) -> Optional[float]:
    try:
        with open(path, "r") as f:
            return float(f.read().strip())
    except Exception:
        return None


def read_voltage(shm_path: str = "/dev/shm/jetracer_voltage") -> Optional[float]:
    return read_float_file(shm_path)


def read_battery_pct(
    shm_path: str = "/dev/shm/jetracer_voltage",
    cells: int = 2,
) -> Optional[float]:
    v = read_voltage(shm_path)
    if v is None:
        return None
    return float(soc_from_voltage(v, cells=cells))


# =========================
# Vehicle / car number
# =========================

def infer_car_number(explicit: Optional[int] = None) -> int:
    """
    If explicit is given, use it. Otherwise parse trailing digits from username.
    Example: "jet3" -> 3. Returns 0 if not found.
    """
    if explicit is not None:
        return max(0, int(explicit))

    username = os.getenv("USER") or os.getenv("USERNAME") or ""
    if not username:
        try:
            import getpass
            username = getpass.getuser()
        except Exception:
            username = ""

    m = re.search(r"(\d+)\s*$", username)
    return int(m.group(1)) if m else 0


# =========================
# Heading delta (NEW)
# =========================

# SHM binary layout written by imu_yaw_delta_writer.py
# struct.pack("fffI", heading_diff, heading_dt, reserved, heading_seq)
_HEADING_FMT = "fffI"
_HEADING_SIZE = struct.calcsize(_HEADING_FMT)


def read_heading_delta(
    shm_path: str = "/dev/shm/jetracer_heading_delta",
) -> Tuple[Optional[float], Optional[float], Optional[int]]:
    """
    Read heading delta information.

    Returns:
        heading_diff (rad)  : Δψ in [-pi, pi]
        heading_dt   (sec)  : integration window (≈ 1/30)
        heading_seq  (uint) : IMU sequence counter
    """
    try:
        with open(shm_path, "rb") as f:
            data = f.read(_HEADING_SIZE)
            if len(data) != _HEADING_SIZE:
                return None, None, None

            heading_diff, heading_dt, _, heading_seq = struct.unpack(
                _HEADING_FMT, data
            )
            return float(heading_diff), float(heading_dt), int(heading_seq)

    except Exception:
        return None, None, None


# =========================
# Deprecated (for safety)
# =========================

def read_qyaw(*args, **kwargs):
    """
    Deprecated.
    Absolute yaw is no longer supported.
    """
    return None
