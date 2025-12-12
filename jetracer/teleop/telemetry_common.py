#!/usr/bin/env python3
"""
Common helpers for JetRacer telemetry.

- Battery voltage is read from /dev/shm/jetracer_voltage (written by battery_monitor service).
- Q-yaw is read from /dev/shm/jetracer_qyaw (written by keyboard_yaw_hold).

No I2C or serial access here.
"""

from __future__ import annotations

import os
import re
from typing import Optional

# SOC curve copied from battery_monitor.py (2S pack, per-cell interpolation)
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


def read_qyaw(shm_path: str = "/dev/shm/jetracer_qyaw") -> Optional[float]:
    return read_float_file(shm_path)

