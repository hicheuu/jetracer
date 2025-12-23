#!/usr/bin/env python3
import multiprocessing
import time
import argparse
import sys
import os

from jetracer.mux.mux import run_mux
from jetracer.mux.joystick import run_joystick
from jetracer.mux.udp_recv import run_udp

def runner(args):
    # Queue for all logs
    log_queue = multiprocessing.Queue()
    stop_event = multiprocessing.Event()

    # Processes
    # Mux
    p_mux = multiprocessing.Process(target=run_mux, args=(log_queue, stop_event))
    p_mux.start()

    # Joystick
    p_joy = multiprocessing.Process(
        target=run_joystick, 
        args=(log_queue, stop_event),
        kwargs={
            "device": args.device,
            "max_throttle": args.max_throttle
        }
    )
    p_joy.start()

    # UDP
    p_udp = multiprocessing.Process(
        target=run_udp,
        args=(log_queue, stop_event, args.speed5_throttle)
    )
    p_udp.start()

    print("[RUNNER] All processes started. Press Ctrl+C to stop.")
    
    current_mode = "joystick" # Default startup mode

    try:
        while True:
            try:
                record = log_queue.get(timeout=0.1)
                
                # Check for mode change
                if record.get("type") == "MODE":
                    current_mode = record["mode"]
                    # print(f"[RUNNER] Mode switched to: {current_mode}") # Optional: redundant if Mux logs it
                    continue

                if record.get("type") == "LOG":
                    src = record["src"]
                    msg = record["msg"]
                    
                    # Log Filtering Logic
                    should_print = False
                    
                    if src == "MUX":
                        # Always show MUX logs (or critical ones)
                        should_print = True
                        
                    elif src == "JOY":
                        # Show Joy logs if mode is joystick or if it's an error/setup message
                        if current_mode == "joystick":
                            should_print = True
                        elif "Error" in msg or "Device" in msg or "stopping" in msg:
                            should_print = True

                    elif src == "UDP":
                        # Show UDP logs only when in UDP mode, except for actual errors
                        if current_mode == "udp":
                            should_print = True
                        elif "Error" in msg or "stopping" in msg:
                            should_print = True
                        # watchdog 등 기타 메시지는 UDP 모드가 아닐 때 숨김

                    if should_print:
                        print(f"[{src}] {msg}")

            except multiprocessing.queues.Empty:
                pass
            
            # Check if any process died unexpectedly
            if not p_mux.is_alive():
                print("[RUNNER] MUX process died. Exiting...")
                break
                
    except KeyboardInterrupt:
        print("\n[RUNNER] Stopping...")
    finally:
        stop_event.set()
        p_mux.join(timeout=1)
        p_joy.join(timeout=1)
        p_udp.join(timeout=1)
        
        # Force kill if stuck
        if p_mux.is_alive(): p_mux.terminate()
        if p_joy.is_alive(): p_joy.terminate()
        if p_udp.is_alive(): p_udp.terminate()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Jetracer Unified Runner")
    parser.add_argument("--speed5-throttle", type=float, default=0.20, help="Throttle for speed=5.0 (UDP)")
    parser.add_argument("--device", default=None, help="Joystick device path")
    parser.add_argument("--max-throttle", type=float, default=0.24, help="Max manual throttle")
    
    args = parser.parse_args()
    
    runner(args)
