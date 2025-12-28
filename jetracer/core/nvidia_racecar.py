import traitlets
from adafruit_servokit import ServoKit
import json
from pathlib import Path

from .racecar import Racecar


def load_config(config_path=None):
    """
    차량 제어에 필요한 실제 설정 파일(JSON)을 로드합니다.
    """
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
        print(f"[jetracer] 설정 파일 읽기 오류: {e}")
        return None


class NvidiaRacecar(Racecar):
    """
    JetRacer용 PCA9685 기반 차량 제어 클래스입니다.
    
    -1.0 ~ 1.0 의 표준 입력을 받아 ESC와 서보의 물리 신호로 변환합니다.
    사용자가 제시한 discontinuous reverse(후진 시작점 점프) 로직을 포함합니다.
    """

    i2c_address = traitlets.Integer(default_value=0x40)
    steering_gain = traitlets.Float(default_value=-0.65)
    steering_offset = traitlets.Float(default_value=0.22)
    steering_channel = traitlets.Integer(default_value=0)
    steering_throttle_gain_left = traitlets.Float(default_value=0.0)
    steering_throttle_gain_right = traitlets.Float(default_value=0.0)
    
    throttle_gain = traitlets.Float(default_value=1.0)
    throttle_channel = traitlets.Integer(default_value=1)
    speed5_throttle = traitlets.Float(default_value=0.20)
    
    # 후진 속도 배율 (기본 0.7로 감속하여 안전성 확보)
    throttle_reverse_gain = traitlets.Float(default_value=0.7)
    
    # 후진 진입을 위한 물리적 시작점 (기능 활성화를 위해 -0.1 등으로 설정)
    throttle_reverse_start = traitlets.Float(default_value=-0.1)
    
    verbose = traitlets.Bool(default_value=False)
    
    INPUT_TO_MS = 2.0
    
    def __init__(self, *args, config_path=None, **kwargs):
        """
        하드웨어를 초기화하고 설정을 적용합니다.
        """
        self._throttle_neutral = 0.12 
        
        config = load_config(config_path)
        if config:
            if "i2c_address" in config:
                kwargs.setdefault("i2c_address", config["i2c_address"])
            
            if "steering" in config:
                steering = config["steering"]
                if "gain" in steering: kwargs["steering_gain"] = steering["gain"]
                if "offset" in steering: kwargs["steering_offset"] = steering["offset"]
                if "channel" in steering: kwargs["steering_channel"] = steering["channel"]
                kwargs.setdefault("steering_throttle_gain_left", steering.get("throttle_gain_left", 0.0))
                kwargs.setdefault("steering_throttle_gain_right", steering.get("throttle_gain_right", 0.0))
            
            if "throttle" in config:
                throttle = config["throttle"]
                kwargs.setdefault("throttle_gain", throttle.get("gain", 1.0))
                kwargs.setdefault("throttle_channel", throttle.get("channel", 1))
                kwargs.setdefault("throttle_reverse_start", throttle.get("reverse_start", -0.1))
                kwargs.setdefault("speed5_throttle", throttle.get("speed5_throttle", 0.20))
                kwargs.setdefault("throttle_reverse_gain", throttle.get("reverse_gain", 0.7))
                self._throttle_neutral = throttle.get("neutral", 0.12)
            
            if "input_to_ms" in config:
                self.INPUT_TO_MS = config["input_to_ms"]
            
            print(f"[jetracer] 외부 설정 파일 로드 완료")
        else:
            print("[jetracer] 기본값 사용")
        
        super().__init__(*args, **kwargs)
        self.kit = ServoKit(channels=16, address=self.i2c_address)
        self.kit._pca.frequency = 60
        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.throttle_channel]
        
        self.steering = 0.0
        self.throttle = 0.0
        
        self._last_printed_steering = None
        self._last_printed_throttle = None
        self._last_physical_throttle = self._throttle_neutral

    @property
    def physical_throttle(self):
        """
        현재 ESC에 전달되고 있는 실제 물리적 스로틀 값(Neutral + Offset)을 반환합니다.
        """
        return self._last_physical_throttle

    @traitlets.observe("steering")
    def _on_steering(self, change):
        """
        스티어링 제어: 중앙점 + (입력 * 게인)
        """
        final_steering = self.steering_offset + (change["new"] * self.steering_gain)
        final_steering = max(-1.0, min(1.0, final_steering))
        self.steering_motor.throttle = final_steering

    @traitlets.observe("throttle")
    def _on_throttle(self, change):
        """
        스로틀 제어: 
        - 전진(>0): 중립점 + 입력 * (최대1.0 - 중립점) * 게인
        - 후진(<0): 후진시작점 + 입력 * (최소-1.0 - 후진시작점) * 게인
        - 정지(=0): 중립점
        """
        input_val = change["new"]
        
        if abs(input_val) < 0.01:
            # 1. 정지 (Neutral)
            final_throttle = self._throttle_neutral
        elif input_val > 0:
            # 2. 전진
            range_fwd = (1.0 - self._throttle_neutral)
            final_throttle = self._throttle_neutral + (input_val * range_fwd * self.throttle_gain)
        else:
            # 3. 후진 (reverse_gain 적용)
            range_rev = (-1.0 - self.throttle_reverse_start)
            final_throttle = self.throttle_reverse_start + (abs(input_val) * range_rev * self.throttle_gain * self.throttle_reverse_gain)
        
        # 물리적 한계 클리핑
        final_throttle = max(-1.0, min(1.0, final_throttle))
        self._last_physical_throttle = final_throttle
        
        # 캘리브레이션용 로그 출력
        rounded_phys = round(final_throttle, 3)
        if getattr(self, "verbose", False) and rounded_phys != self._last_printed_throttle:
            print(f"[motor] target={input_val:+.3f} | physical_esc={final_throttle:.3f}")
            self._last_printed_throttle = rounded_phys
            
        self.throttle_motor.throttle = final_throttle
