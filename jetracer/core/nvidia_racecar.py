import traitlets
from adafruit_servokit import ServoKit
import json
from pathlib import Path

from .racecar import Racecar


def load_config(config_path=None):
    """
    차량 제어에 필요한 실제 설정 파일(JSON)을 로드합니다.
    최상위 영역의 config/nvidia_racecar_config.json만 참조합니다.
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
    
    표준화된 입력(-1.0 ~ 1.0)을 물리적 하드웨어 신호로 변환합니다.
    직관적인 캘리브레이션을 위해 기본 게인은 1.0(1:1 매핑)을 사용합니다.
    """

    i2c_address = traitlets.Integer(default_value=0x40)
    steering_gain = traitlets.Float(default_value=-0.65)
    steering_offset = traitlets.Float(default_value=0.22)
    steering_channel = traitlets.Integer(default_value=0)
    
    # 기본 게인을 1.0으로 설정하여 입력값과 출력 변화량이 1:1이 되도록 함
    throttle_gain = traitlets.Float(default_value=1.0)
    throttle_offset = traitlets.Float(default_value=0.0)
    throttle_channel = traitlets.Integer(default_value=1)
    
    INPUT_TO_MS = 2.0
    
    def __init__(self, *args, config_path=None, **kwargs):
        """
        차량 하드웨어 초기화 및 설정 로드.
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
            
            if "throttle" in config:
                throttle = config["throttle"]
                # 설정 파일에 값이 있으면 적용, 없으면 기본값(1.0) 유지
                kwargs.setdefault("throttle_gain", throttle.get("gain", 1.0))
                kwargs.setdefault("throttle_offset", throttle.get("offset", 0.0))
                kwargs.setdefault("throttle_channel", throttle.get("channel", 1))
                self._throttle_neutral = throttle.get("neutral", 0.12)
            
            if "input_to_ms" in config:
                self.INPUT_TO_MS = config["input_to_ms"]
            
            print(f"[jetracer] 외부 설정 파일 로드 완료")
        else:
            print("[jetracer] 설정 파일 없음 - 기본 제어값 사용")
        
        super().__init__(*args, **kwargs)
        self.kit = ServoKit(channels=16, address=self.i2c_address)
        self.kit._pca.frequency = 60
        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.throttle_channel]
        
        self.steering = 0.0
        self.throttle = 0.0
        
        self._last_printed_steering = None
        self._last_printed_throttle = None

    @traitlets.observe("steering")
    def _on_steering(self, change):
        """
        조향 제어: offset + (input * gain)
        """
        final_steering = self.steering_offset + (change["new"] * self.steering_gain)
        final_steering = max(-1.0, min(1.0, final_steering))
        self.steering_motor.throttle = final_steering

    @traitlets.observe("throttle")
    def _on_throttle(self, change):
        """
        스로틀 제어: neutral + (input * gain) + offset
        
        실제 모터에 들어가는 최종 물리 값(raw_thr)을 로그로 출력하여 
        사용자가 직관적으로 최저/최대 스로틀을 찾을 수 있게 합니다.
        """
        input_val = change["new"]
        
        if abs(input_val) < 0.01:
            final_throttle = self._throttle_neutral
        else:
            # gain이 1.0이면 neutral에 입력한 delta가 그대로 더해집니다.
            final_throttle = self._throttle_neutral + (input_val * self.throttle_gain) + self.throttle_offset
        
        final_throttle = max(-1.0, min(1.0, final_throttle))
        
        # 캘리브레이션을 위한 물리적 ESC 출력값 로그 출력 (중요)
        rounded_phys = round(final_throttle, 3)
        if rounded_phys != self._last_printed_throttle:
            print(f"[motor] target={input_val:+.3f} | physical_esc={final_throttle:.3f}")
            self._last_printed_throttle = rounded_phys
            
        self.throttle_motor.throttle = final_throttle
