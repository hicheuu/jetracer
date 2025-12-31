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

def run_telemetry(stop_event, server_ip="192.168.0.100", server_port=5560, hz=30.0, car_number=None, battery_shm_path="/dev/shm/jetracer_voltage", verbose=False):
    vehicle_id = infer_car_number(car_number)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (server_ip, server_port)

    interval = 1.0 / max(0.1, float(hz))

    if verbose:
        print(f"[uplink] started. Server={server_ip}:{server_port} HZ={hz} ID={vehicle_id}")

    try:
        while not stop_event.is_set():
            start_time = time.monotonic()
            
            voltage = read_voltage(battery_shm_path) or 0.0
            
            # Packet: (ID:int, Voltage:float) -> 4 + 4 = 8 bytes
            pkt = struct.pack(FMT_UPLINK, int(vehicle_id), float(voltage))
            sock.sendto(pkt, target)
            
            if verbose:
                # 1초에 한 번 정도만 출력 (너무 잦음 방지)
                now_s = int(time.time())
                if now_s % 2 == 0:
                    # 중복 출력 방지용 로직 (실제로는 sleep 때문에 크게 문제 없지만 완벽을 기함)
                    pass 
            
            # 2초마다 한 번씩 전압 출력하여 데이터 송신 확인 가능하게 함
            if verbose and (int(time.monotonic()) % 2 == 0):
                print(f"[uplink] Sending telemetry: {voltage:.2f}V (ID:{vehicle_id})")

            # 타이머 조절
            elapsed = time.monotonic() - start_time
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        if verbose:
            print("[uplink] stopping")
        sock.close()

def main():
    args = build_parser().parse_args()
    import multiprocessing
    stop_event = multiprocessing.Event()
    try:
        run_telemetry(
            stop_event,
            server_ip=args.server_ip,
            server_port=args.server_port,
            hz=args.hz,
            car_number=args.car_number,
            battery_shm_path=args.battery_shm_path,
            verbose=args.verbose
        )
    except KeyboardInterrupt:
        stop_event.set()

if __name__ == "__main__":
    main()