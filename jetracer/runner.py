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
                        # Show Joy logs if mode is joystick or if it's a device/error message (not continuous)
                        # To be safe, let's trust the 'mode' for continuous data. 
                        # But button presses (events) should arguably always be seen? 
                        # User request: "joystick의 로그를 보고... udp_recv일때 udp_recv로그를 보도록"
                        # This implies filtering the *continuous* control logs.
                        # Critical logs (errors, device info) should probably pass through.
                        # For simplicity, assuming 'msg' doesn't have metadata about priority.
                        # Heuristic: if it's an error or initialization, show it.
                        if "steer=" not in msg and "max_throttle" not in msg:
                             # Likely setup/error message
                             should_print = True
                        elif current_mode == "joystick":
                             should_print = True

                    elif src == "UDP":
                        if "steer=" not in msg:
                             # Setup/error
                             should_print = True
                        elif current_mode == "udp":
                             should_print = True

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
