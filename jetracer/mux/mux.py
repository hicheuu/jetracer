import socket
import json
import time
import os
import multiprocessing
import argparse
import csv
from datetime import datetime

from jetracer.core import NvidiaRacecar


# =========================
# UDS 설정
# =========================
SOCK_PATH = "/tmp/jetracer_ctrl.sock"

# =========================
# 타임아웃 설정
# =========================
JOY_TIMEOUT = 0.5
UDP_TIMEOUT = 1.2


def speed_to_normalized_throttle(speed: float, 
                                 speed1_phys: float, 
                                 speed5_phys: float, 
                                 neutral: float, 
                                 gain: float) -> float:
    """
    속도(0.0 ~ 5.0)를 NvidiaRacecar가 사용하는 정규화된 스로틀 값(0.0 ~ 1.0)으로 변환합니다.
    
    NvidiaRacecar의 공식(전진): 
    phys = neutral + (norm * (1.0 - neutral) * gain)
    
    역산 공식:
    norm = (phys - neutral) / ((1.0 - neutral) * gain)
    """
    if speed <= 0.0:
        return 0.0
    
    slope = (speed5_phys - speed1_phys) / (5.0 - 1.0)
    intercept = speed1_phys - slope * 1.0
    target_phys = slope * speed + intercept
    
    denominator = (1.0 - neutral) * gain
    if abs(denominator) < 1e-5:
        return 0.0
        
    normalized = (target_phys - neutral) / denominator
    
    return max(0.0, min(1.0, normalized))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--speed5-throttle",
        type=float,
        default=None,
        help="속도 5.0일 때의 물리적 스로틀 목표값 (None인 경우 config에서 로드)"
    )
    return parser.parse_args()


def run_mux(log_queue, stop_event, speed5_throttle, log_calibration=False, verbose_motor=False):
    # 기존 소켓 제거
    if os.path.exists(SOCK_PATH):
        try:
            os.unlink(SOCK_PATH)
        except OSError:
            pass

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(SOCK_PATH)
    sock.setblocking(False)

    car = NvidiaRacecar(verbose=verbose_motor)
    
    # 하드웨어 파라미터 추출
    ESC_NEUTRAL = car._throttle_neutral
    THR_GAIN = car.throttle_gain
    
    # 조향-스로틀 보정 게인 (설정 파일에서 로드)
    steer_thr_gain_left = car.steering_throttle_gain_left
    steer_thr_gain_right = car.steering_throttle_gain_right
    
    # SPEED_5_PHYS 설정: 인자로 받지 못한 경우(None) Car 설정값 사용
    SPEED_5_PHYS = speed5_throttle if speed5_throttle is not None else car.speed5_throttle
    SPEED_1_PHYS = SPEED_5_PHYS - 0.01  
    
    # 캐리브레이션 로깅 설정
    csv_file = None
    csv_writer = None
    if log_calibration:
        os.makedirs("logs", exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = f"logs/calibration_{timestamp}.csv"
        csv_file = open(log_path, 'w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["timestamp", "type", "value", "direction", "obs_value"]) # 헤더
        log_queue.put({"type": "LOG", "src": "MUX", "msg": f"Calibration logging enabled: {log_path}"})

    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"Mapped Speed Mapping: Neutral={ESC_NEUTRAL:.3f}, Gain={THR_GAIN:.2f}"})
    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"Loaded Steer Gains: L={steer_thr_gain_left:.3f}, R={steer_thr_gain_right:.3f}"})
    
    car.steering = 0.0
    car.throttle = 0.0

    mode = "joystick"
    estop = False

    last_joy = None
    last_udp = None
    last_log_time = 0.0

    log_queue.put({"type": "MODE", "mode": mode})

    try:
        while not stop_event.is_set():
            try:
                data, _ = sock.recvfrom(512)
                msg = json.loads(data.decode())
                src = msg.get("src")

                if msg.get("event") == "toggle":
                    mode = "udp" if mode == "joystick" else "joystick"
                    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"MODE → {mode}"})
                    log_queue.put({"type": "MODE", "mode": mode})
                    continue

                if msg.get("event") == "estop":
                    estop = not estop
                    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"ESTOP {'ON' if estop else 'OFF'}"})
                    continue

                if msg.get("event") == "update_speed5":
                    SPEED_5_PHYS = msg["val"]
                    SPEED_1_PHYS = SPEED_5_PHYS - 0.01
                    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"Remote update: SPEED_5_PHYS → {SPEED_5_PHYS:.3f}"})
                    continue

                if msg.get("event") == "speed5_adjust":
                    delta = msg.get("delta", 0.0)
                    SPEED_5_PHYS += delta
                    SPEED_1_PHYS = SPEED_5_PHYS - 0.01
                    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"SPEED_5_PHYS (Auto) → {SPEED_5_PHYS:.3f} ({delta:+.3f})"})
                    if csv_writer:
                        cv_dir = "+" if delta > 0 else "-"
                        csv_writer.writerow([time.time(), "auto_adjust", SPEED_5_PHYS, cv_dir, ""])
                    continue

                if msg.get("event") == "speed5_up":
                    step = 0.01 if mode == "joystick" else 0.001
                    SPEED_5_PHYS += step
                    SPEED_1_PHYS = SPEED_5_PHYS - 0.01
                    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"SPEED_5_PHYS → {SPEED_5_PHYS:.3f} (+{step})"})
                    if csv_writer:
                        csv_writer.writerow([time.time(), "adjust", SPEED_5_PHYS, "+", ""])
                    continue

                if msg.get("event") == "speed5_down":
                    step = 0.01 if mode == "joystick" else 0.001
                    SPEED_5_PHYS -= step
                    SPEED_1_PHYS = SPEED_5_PHYS - 0.01
                    log_queue.put({"type": "LOG", "src": "MUX", "msg": f"SPEED_5_PHYS → {SPEED_5_PHYS:.3f} (-{step})"})
                    if csv_writer:
                        csv_writer.writerow([time.time(), "adjust", SPEED_5_PHYS, "-", ""])
                    continue

                if msg.get("event") == "steer_gain_up":
                    step = 0.001
                    if car.steering < -0.1:
                        steer_thr_gain_left += step
                        log_queue.put({"type": "LOG", "src": "MUX", "msg": f"Steer Gain [LEFT] → {steer_thr_gain_left:.3f}"})
                    elif car.steering > 0.1:
                        steer_thr_gain_right += step
                        log_queue.put({"type": "LOG", "src": "MUX", "msg": f"Steer Gain [RIGHT] → {steer_thr_gain_right:.3f}"})
                    else:
                        log_queue.put({"type": "LOG", "src": "MUX", "msg": "Steering neutral. Turn to adjust direction-specific gain."})
                    continue

                if msg.get("event") == "steer_gain_down":
                    step = 0.001
                    if car.steering < -0.1:
                        steer_thr_gain_left = max(0.0, steer_thr_gain_left - step)
                        log_queue.put({"type": "LOG", "src": "MUX", "msg": f"Steer Gain [LEFT] → {steer_thr_gain_left:.3f}"})
                    elif car.steering > 0.1:
                        steer_thr_gain_right = max(0.0, steer_thr_gain_right - step)
                        log_queue.put({"type": "LOG", "src": "MUX", "msg": f"Steer Gain [RIGHT] → {steer_thr_gain_right:.3f}"})
                    else:
                        log_queue.put({"type": "LOG", "src": "MUX", "msg": "Steering neutral. Turn to adjust direction-specific gain."})
                    continue

                msg["ts"] = time.time()
                if src == "joystick":
                    last_joy = msg
                elif src == "udp":
                    last_udp = msg
                    if csv_writer and "speed" in msg:
                        obs_sp = msg.get("obs_speed", 0.0)
                        # value 컬럼에 cmd["speed"] 대신 현재 조이스틱 기준인 SPEED_5_PHYS를 기록
                        csv_writer.writerow([time.time(), "speed", SPEED_5_PHYS, "", obs_sp])

            except BlockingIOError:
                pass
            except Exception as e:
               log_queue.put({"type": "LOG", "src": "MUX", "msg": f"Error: {e}"})

            now = time.time()

            if estop:
                car.steering = 0.0
                car.throttle = 0.0
                time.sleep(0.01)
                continue

            cmd = None
            src_used = None

            if mode == "udp" and last_udp and now - last_udp["ts"] < UDP_TIMEOUT:
                cmd = last_udp
                src_used = "UDP"
            elif mode == "joystick" and last_joy and now - last_joy["ts"] < JOY_TIMEOUT:
                cmd = last_joy
                src_used = "JOY"

            if cmd:
                car.steering = cmd["steer"]
                
                if "speed" in cmd:
                    # 새로운 NvidiaRacecar 공식에 맞춘 정교한 역산 적용
                    car.throttle = speed_to_normalized_throttle(
                        cmd["speed"],
                        SPEED_1_PHYS,
                        SPEED_5_PHYS,
                        ESC_NEUTRAL,
                        THR_GAIN
                    )
                else:
                    # 조이스틱 입력 (-1.0 ~ 1.0) 처리
                    # 조이스틱의 1.0(최대)이 SPEED_5_PHYS에 도달하도록 스케일링
                    # speed_to_normalized_throttle 함수는 0~5 범위를 받으므로 
                    # 조이스틱 입력을 0~5 범위로 매핑하여 재활용
                    joy_throttle = cmd["throttle"]
                    if joy_throttle > 0:
                        # 0.0~1.0 조이스틱 입력을 0.0~5.0 속도로 변환하여 동일한 물리 타겟팅 적용
                        virtual_speed = joy_throttle * 5.0
                        car.throttle = speed_to_normalized_throttle(
                            virtual_speed,
                            SPEED_1_PHYS,
                            SPEED_5_PHYS,
                            ESC_NEUTRAL,
                            THR_GAIN
                        )
                    else:
                        # 후진은 NvidiaRacecar의 내부 공식(REVERSE_START base)을 따릅니다.
                        car.throttle = joy_throttle
                
                # 조향 시 감속 방지를 위한 보정 게인 적용 (좌/우 개별 적용)
                if abs(car.steering) > 0.1 and abs(car.throttle) > 0.01:
                    # 좌조향(steering < 0), 우조향(steering > 0)에 맞춰 게인 선택
                    gain = steer_thr_gain_left if car.steering < 0 else steer_thr_gain_right
                    compensation = abs(car.steering) * gain
                    
                    if car.throttle > 0:
                        car.throttle = min(1.0, car.throttle + compensation)
                    else:
                        car.throttle = max(-1.0, car.throttle - compensation)
                
            else:
                car.steering = 0.0
                car.throttle = 0.0
                src_used = "IDLE"

            # 0.5초 주기로 제어 상태 로그 출력
            if now - last_log_time > 0.5:
                # physical_throttle은 NvidiaRacecar의 내부 물리 출력값
                log_queue.put({
                    "type": "LOG", 
                    "src": "MUX", 
                    "msg": f"[{src_used}] steer={car.steering:+.2f} thr={car.physical_throttle:.3f} (phys)"
                })
                last_log_time = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        log_queue.put({"type": "LOG", "src": "MUX", "msg": "--- 최종 가동 요약 ---"})
        log_queue.put({"type": "LOG", "src": "MUX", "msg": f"최종 SPEED_5_PHYS: {SPEED_5_PHYS:.4f}"})
        log_queue.put({"type": "LOG", "src": "MUX", "msg": f"최종 Steer Gain (L/R): {steer_thr_gain_left:.3f} / {steer_thr_gain_right:.3f}"})
        log_queue.put({"type": "LOG", "src": "MUX", "msg": "----------------------"})
        car.steering = 0.0
        car.throttle = 0.0
        sock.close()
        if csv_file:
            csv_file.close()
        if os.path.exists(SOCK_PATH):
            os.unlink(SOCK_PATH)
        log_queue.put({"type": "LOG", "src": "MUX", "msg": "stopped"})


if __name__ == "__main__":
    args = parse_args()
    
    class PrintQueue:
        def put(self, item):
            if item.get("type") == "LOG":
                print(f"[{item['src']}] {item['msg']}")
            elif item.get("type") == "MODE":
                 print(f"[{item['mode']}] MODE CHANGE")

    stop = multiprocessing.Event()
    try:
        run_mux(PrintQueue(), stop, args.speed5_throttle)
    except KeyboardInterrupt:
        stop.set()
