import traitlets
from adafruit_servokit import ServoKit
import json
from pathlib import Path

from .racecar import Racecar


def load_config(config_path=None):
    """
    차량 제어에 필요한 실제 설정 파일(JSON)을 로드합니다.
    
    최상위 config 디렉토리에 있는 nvidia_racecar_config.json을 우선적으로 참조하며,
    파일이 존재하지 않을 경우 None을 반환합니다.
    """
    if config_path is None:
        current_file = Path(__file__).resolve()
        # jetracer/core/nvidia_racecar.py -> jetracer -> 프로젝트 루트 -> config
        project_root = current_file.parent.parent.parent
        config_path = project_root / "config" / "nvidia_racecar_config.json"
    
    config_path = Path(config_path)
    
    if not config_path.exists():
        # 사용자 요청에 따라 패키지 내부 설정은 참조하지 않음
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
    
    표준화된 입력(-1.0 ~ 1.0)을 받아 실제 하드웨어 신호로 변환하는 역할을 담당합니다.
    """

    i2c_address = traitlets.Integer(default_value=0x40)
    steering_gain = traitlets.Float(default_value=-0.65)
    steering_offset = traitlets.Float(default_value=0.22)
    steering_channel = traitlets.Integer(default_value=0)
    throttle_gain = traitlets.Float(default_value=0.8)
    throttle_offset = traitlets.Float(default_value=0.0)
    throttle_channel = traitlets.Integer(default_value=1)
    
    INPUT_TO_MS = 2.0
    
    def __init__(self, *args, config_path=None, **kwargs):
        """
        차량 하드웨어를 초기화합니다.
        
        설정 파일이 로드되면 중립점과 게인 등을 해당 값으로 갱신합니다.
        """
        # 기본 중립값 (설정 파일 로드 실패 시의 안전 장치)
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
                kwargs.setdefault("throttle_gain", throttle.get("gain", 0.8))
                kwargs.setdefault("throttle_offset", throttle.get("offset", 0.0))
                kwargs.setdefault("throttle_channel", throttle.get("channel", 1))
                self._throttle_neutral = throttle.get("neutral", 0.12)
            
            if "input_to_ms" in config:
                self.INPUT_TO_MS = config["input_to_ms"]
            
            print(f"[jetracer] 외부 설정 파일 로드 완료")
        else:
            print("[jetracer] 설정 파일을 찾을 수 없어 기본값을 사용합니다.")
        
        super().__init__(*args, **kwargs)
        self.kit = ServoKit(channels=16, address=self.i2c_address)
        self.kit._pca.frequency = 60
        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.throttle_channel]
        
        # 중립 상태로 시작
        self.steering = 0.0
        self.throttle = 0.0
        
        self._last_printed_steering = None
        self._last_printed_throttle = None

    @traitlets.observe("steering")
    def _on_steering(self, change):
        """
        스티어링 제어값을 물리적 신호로 변환합니다.
        공식: 중앙오프셋 + (정규모드입력 * 게인)
        """
        final_steering = self.steering_offset + (change["new"] * self.steering_gain)
        final_steering = max(-1.0, min(1.0, final_steering))
        self.steering_motor.throttle = final_steering

    @traitlets.observe("throttle")
    def _on_throttle(self, change):
        """
        스로틀 제어값을 물리적 신호로 변환합니다.
        
        정지 상태(0.0)일 때는 하드웨어 중립값을 정확히 출력하며,
        동작 시에는 중립점을 기준으로 게인과 오프셋을 더해 최종 출력을 결정합니다.
        """
        input_val = change["new"]
        
        # 0.01 이하의 미세 신호는 정지로 간주하여 중립값 출력
        if abs(input_val) < 0.01:
            final_throttle = self._throttle_neutral
        else:
            # 중립점 + (입력값 * 게인) + 미세조정오프셋
            final_throttle = self._throttle_neutral + (input_val * self.throttle_gain) + self.throttle_offset
        
        final_throttle = max(-1.0, min(1.0, final_throttle))
        
        # 로깅 (값이 크게 변할 때만 m/s 단위로 표시)
        speed_ms = input_val * self.INPUT_TO_MS
        rounded = round(speed_ms, 1)
        if rounded != self._last_printed_throttle:
            # print(f"[motor] speed target: {speed_ms:.2f} m/s")
            self._last_printed_throttle = rounded
            
        self.throttle_motor.throttle = final_throttle
