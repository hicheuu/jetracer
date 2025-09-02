#!/usr/bin/env python3
# OLED: Adafruit_SSD1306 128x32  |  Sensor: INA219
# 표시: 배터리 %, 전압, Wi-Fi SSID, IP (Wi-Fi 우선, 없으면 유선)

import time, re, socket, subprocess
import board, busio
from PIL import Image, ImageDraw, ImageFont
import Adafruit_SSD1306
from adafruit_ina219 import INA219

# ---------------- OLED ----------------
RST = None
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST, i2c_bus=1, gpio=1)
disp.begin()
disp.clear()
disp.display()

width, height = disp.width, disp.height
image = Image.new("1", (width, height))
draw  = ImageDraw.Draw(image)
font  = ImageFont.load_default()  # 작고 선명

# ---------------- INA219 ----------------
i2c_bus = busio.I2C(board.SCL, board.SDA)
INA_ADDR = 0x40  # 필요 시 0x42
ina = INA219(i2c_bus, addr=INA_ADDR)

# ---------------- 배터리 % (4셀 3.0~4.2V 기준) ----------------
def battery_percent(voltage, cells=4, v_empty=3.0, v_full=4.2):
    vpc = voltage / cells
    pct = (vpc - v_empty) / (v_full - v_empty) * 100.0
    return max(0, min(100, pct))

# ---------------- 네트워크 유틸 ----------------
def _run(cmd):
    # 절대경로 사용 + text=True
    return subprocess.check_output(cmd, text=True).strip()

def get_wifi_ssid():
    # 1) nmcli (NetworkManager)
    try:
        out = _run(["/usr/bin/nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"])
        for line in out.splitlines():
            if line.startswith("yes:"):
                return line.split(":", 1)[1] or None
    except Exception:
        pass
    # 2) iwgetid
    for path in ("/usr/sbin/iwgetid", "/sbin/iwgetid"):
        try:
            ssid = _run([path, "-r"])
            if ssid:
                return ssid
        except Exception:
            pass
    return None

def _ip_from_iface(iface):
    # ip -4 addr show wlan0 | grep 'inet ' | awk '{print $2}' -> 192.168.x.x
    for ipbin in ("/usr/sbin/ip", "/sbin/ip", "/usr/bin/ip"):
        try:
            out = _run([ipbin, "-4", "addr", "show", iface])
            m = re.search(r"inet\s+(\d+\.\d+\.\d+\.\d+)", out)
            if m:
                return m.group(1)
        except Exception:
            continue
    return None

def get_ip():
    # Wi-Fi 우선, 없으면 유선
    for iface in ("wlan0", "wlp1s0", "wlp2s0", "eth0"):
        ip = _ip_from_iface(iface)
        if ip:
            return ip
    # 최후수단: 외부로 더미 연결해서 로컬 IP 추출
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.2)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return None

def draw_oled(bat_pct, voltage, ssid, ip):
    draw.rectangle((0, 0, width, height), outline=0, fill=0)
    # 1행: 배터리 % + 전압
    draw.text((0, 1),  f"Bat:{bat_pct:3.0f}% ({voltage:4.1f}V)", font=font, fill=255)
    # 2행: Wi-Fi 상태
   # wifi_txt = f"WiFi:{ssid}" if ssid else "WiFi:OFF"
   # draw.text((0, 12), wifi_txt, font=font, fill=255)
    # 3행: IP
    ip_txt = ip if ip else "-"
    draw.text((0, 13), f"IP:{ip_txt}", font=font, fill=255)
    disp.image(image)
    disp.display()

# ---------------- 메인 루프 ----------------
try:
    while True:
        V   = ina.bus_voltage
        pct = battery_percent(V)         # 4셀 기준
        ssid = get_wifi_ssid()
        ip   = get_ip()

        # 콘솔 로그
        print(f"Battery={pct:.0f}% ({V:.2f}V) | WiFi={ssid or 'OFF'} | IP={ip or '-'}")

        draw_oled(pct, V, ssid, ip)
        time.sleep(2)

except KeyboardInterrupt:
    disp.clear()
    disp.display()
    print("종료")

