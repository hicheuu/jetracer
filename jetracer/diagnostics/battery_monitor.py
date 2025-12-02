#!/usr/bin/env python3
# JetRacer AI Kit (2S2P, 8.4V full) Battery Monitor
# INA219 @0x42  +  SSD1306 128x32 @0x3C

import re
import socket
import subprocess
import time

import Adafruit_SSD1306
import board
import busio
from PIL import Image, ImageDraw, ImageFont
from adafruit_ina219 import INA219

# ---------- OLED ----------
disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1)
disp.begin()
disp.clear()
disp.display()

width, height = disp.width, disp.height
image = Image.new("1", (width, height))
draw = ImageDraw.Draw(image)
font = ImageFont.load_default()

# ---------- INA219 ----------
i2c = busio.I2C(board.SCL, board.SDA)
ina = INA219(i2c, addr=0x42)

# ---------- SOC (2S, per-cell 테이블 보간) ----------
SOC_TABLE = [(4.20, 100), (4.00, 85), (3.85, 60), (3.70, 40), (3.50, 20), (3.30, 10), (3.00, 0)]


def soc_from_voltage(pack_v, cells=2):
    vpc = pack_v / cells
    if vpc >= 4.20:
        return 100
    if vpc <= 3.00:
        return 0
    for (vh, sh), (vl, sl) in zip(SOC_TABLE, SOC_TABLE[1:]):
        if vl <= vpc <= vh:
            t = (vpc - vl) / (vh - vl)
            return int(sl + t * (sh - sl))
    return 0


# ---------- 네트워크 (선택 표시) ----------
def _run(cmd):
    return subprocess.check_output(cmd, text=True).strip()


def get_wifi_ssid():
    try:
        out = _run(["/usr/bin/nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"])
        for line in out.splitlines():
            if line.startswith("yes:"):
                return line.split(":", 1)[1] or None
    except Exception:
        pass
    for path in ("/usr/sbin/iwgetid", "/sbin/iwgetid"):
        try:
            s = _run([path, "-r"])
            if s:
                return s
        except Exception:
            pass
    return None


def _ip_from_iface(iface):
    for ipbin in ("/usr/sbin/ip", "/sbin/ip", "/usr/bin/ip"):
        try:
            out = _run([ipbin, "-4", "addr", "show", iface])
            m = re.search(r"inet\s+(\d+\.\d+\.\d+\.\d+)", out)
            if m:
                return m.group(1)
        except Exception:
            pass
    return None


def get_ip():
    for iface in ("wlan0", "wlp1s0", "wlp2s0", "eth0"):
        ip = _ip_from_iface(iface)
        if ip:
            return ip
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.2)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return None


def draw_oled(pct, vpack, ssid, ip):
    draw.rectangle((0, 0, width, height), outline=0, fill=0)
    draw.text((0, 0), f"Bat:{pct:3d}% ({vpack:4.2f}V)", font=font, fill=255)
    draw.text((0, 12), f"WiFi:{ssid or 'OFF'}", font=font, fill=255)
    draw.text((0, 22), f"IP:{ip or '-'}", font=font, fill=255)
    disp.image(image)
    disp.display()


try:
    while True:
        vpack = ina.bus_voltage + (ina.shunt_voltage / 1000.0)
        pct = soc_from_voltage(vpack, cells=2)
        ssid = get_wifi_ssid()
        ip = get_ip()
        print(f"Battery={pct}% ({vpack:.2f}V) | WiFi={ssid or 'OFF'} | IP={ip or '-'}")
        
        # 다른 프로세스(Racecar)가 읽을 수 있도록 전압을 파일(RAM Disk)에 기록
        try:
            with open("/dev/shm/jetracer_voltage", "w") as f:
                f.write(f"{vpack:.4f}")
        except Exception:
            pass

        draw_oled(pct, vpack, ssid, ip)
        time.sleep(2)
except KeyboardInterrupt:
    disp.clear()
    disp.display()


