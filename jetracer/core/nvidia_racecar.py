import traitlets
from adafruit_servokit import ServoKit
import json
from pathlib import Path

from .racecar import Racecar


def load_config(config_path=None):
    """JSON 설정 파일을 로드하여 딕셔너리로 반환합니다."""
    if config_path is None:
        current_file = Path(__file__).resolve()
        project_root = current_file.parent.parent.parent
        config_path = project_root / "config" / "nvidia_racecar_config.json"
    
    config_path = Path(config_path)
    if not config_path.exists():
        return None
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = json.load(f)
        return config
    except (json.JSONDecodeError, IOError) as e:
        print(f"[jetracer] 설정 파일 로드 실패: {e}")
        return None


class NvidiaRacecar(Racecar):
    """JetRacer용 PCA9685 기반 차량 제어 클래스 (Normalized Control)."""

    i2c_address = traitlets.Integer(default_value=0x40)
    steering_gain = traitlets.Float(default_value=-0.65)
    steering_offset = traitlets.Float(default_value=0.22)
    steering_channel = traitlets.Integer(default_value=0)
    throttle_gain = traitlets.Float(default_value=0.8)
    throttle_offset = traitlets.Float(default_value=0.0)
    throttle_channel = traitlets.Integer(default_value=1)
    
    # 입력값 → 속도 변환계수 (단순 표시용)
    INPUT_TO_MS = 2.0
    
    def __init__(self, *args, config_path=None, **kwargs):
        config = load_config(config_path)
        if config:
            if "i2c_address" in config:
                kwargs.setdefault("i2c_address", config["i2c_address"])
            
            if "steering" in config:
                steering = config["steering"]
                if "gain" in steering: kwargs["steering_gain"] = steering["gain"]
                if "offset" in steering: kwargs["steering_offset"] = steering["offset"]
                if "channel" in steering: kwargs["steering_channel"] = steering["channel"]
            
            if "throttle" in config:
                throttle = config["throttle"]
                kwargs.setdefault("throttle_gain", throttle.get("gain", NvidiaRacecar.throttle_gain.default_value))
                kwargs.setdefault("throttle_offset", throttle.get("offset", NvidiaRacecar.throttle_offset.default_value))
                kwargs.setdefault("throttle_channel", throttle.get("channel", NvidiaRacecar.throttle_channel.default_value))
                self._throttle_neutral = throttle.get("neutral", 0.12)
            else:
                self._throttle_neutral = 0.12
            
            if "input_to_ms" in config:
                self.INPUT_TO_MS = config["input_to_ms"]
            
            print(f"[jetracer] 설정 로드됨: {config_path or '기본 경로'}")
        else:
            self._throttle_neutral = 0.12
            self.INPUT_TO_MS = 2.0
            print("[jetracer] 기본값 사용")
        
        super().__init__(*args, **kwargs)
        self.kit = ServoKit(channels=16, address=self.i2c_address)
        self.kit._pca.frequency = 60
        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.throttle_channel]
        
        # 초기화 파라미터 적용
        self.steering = 0.0
        self.throttle = 0.0
        
        self._last_printed_steering = None
        self._last_printed_throttle = None

    @traitlets.observe("steering")
    def _on_steering(self, change):
        """
        입력값(-1.0 ~ 1.0)을 물리적 스티어링 값으로 변환.
        공식: offset + (input * gain)
        """
        final_steering = self.steering_offset + (change["new"] * self.steering_gain)
        
        # 클리핑
        final_steering = max(-1.0, min(1.0, final_steering))
        
        # 로그 출력
        rounded = round(final_steering, 2)
        if rounded != self._last_printed_steering:
            # print(f"[motor] steering={final_steering:+.3f}")
            self._last_printed_steering = rounded
        
        self.steering_motor.throttle = final_steering

    @traitlets.observe("throttle")
    def _on_throttle(self, change):
        """
        입력값(-1.0 ~ 1.0)을 물리적 스로틀 값으로 변환.
        공식: neutral + (input * gain) + offset
        """
        # 정지 시 무조건 중립값 출력
        if abs(change["new"]) < 0.01:
            final_throttle = self._throttle_neutral
        else:
            final_throttle = self._throttle_neutral + (change["new"] * self.throttle_gain) + self.throttle_offset
        
        # 클리핑
        final_throttle = max(-1.0, min(1.0, final_throttle))
        
        # 로그 출력 (m/s 단위 표시)
        speed_ms = change["new"] * self.INPUT_TO_MS
        rounded = round(speed_ms, 2)
        if rounded != self._last_printed_throttle:
            print(f"[motor] target_speed={speed_ms:+.3f} m/s (raw_thr={final_throttle:.3f})")
            self._last_printed_throttle = rounded
            
        self.throttle_motor.throttle = final_throttle
