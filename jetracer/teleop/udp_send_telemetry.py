#!/usr/bin/env python3
import argparse
import socket
import struct
import time

from jetracer.teleop.telemetry_common import (
    infer_car_number,
    read_voltage,
)

FMT_UPLINK = "!if"
PKT_SIZE = struct.calcsize(FMT_UPLINK)

def build_parser():
    p = argparse.ArgumentParser()
    p.add_argument("--server-ip", default="192.168.0.100")
    p.add_argument("--server-port", type=int, default=5560)
    p.add_argument("--hz", type=float, default=30.0)
    p.add_argument("--car-number", type=int, default=None)
    p.add_argument("--battery-shm-path", default="/dev/shm/jetracer_voltage")
    p.add_argument("--verbose", action="store_true")
    return p

def main():
    args = build_parser().parse_args()
    vehicle_id = infer_car_number(args.car_number)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.server_ip, args.server_port)

    interval = 1.0 / max(0.1, float(args.hz))

    if args.verbose:
        print(f"[uplink] started. Server={args.server_ip}:{args.server_port} HZ={args.hz}")

    try:
        while True:
            start_time = time.monotonic()
            
            voltage = read_voltage(args.battery_shm_path) or 0.0
            
            # Packet: (ID:int, Voltage:float) -> 4 + 4 = 8 bytes
            pkt = struct.pack(FMT_UPLINK, int(vehicle_id), float(voltage))
            sock.sendto(pkt, target)
            
            if args.verbose:
                print(f"UDP send: voltage={voltage:.2f}V")

            # 타이머 조절
            elapsed = time.monotonic() - start_time
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

if __name__ == "__main__":
    main()