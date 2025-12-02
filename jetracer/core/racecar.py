import traitlets


class Racecar(traitlets.HasTraits):
    """스티어 및 스로틀 값을 traitlets 로 관리하는 기본 차량 모델."""

    steering = traitlets.Float()
    throttle = traitlets.Float()

    @traitlets.validate("steering")
    def _clip_steering(self, proposal):
        value = proposal["value"]
        if value > 1.0:
            return 1.0
        if value < -1.0:
            return -1.0
        return value

    @traitlets.validate("throttle")
    def _clip_throttle(self, proposal):
        value = proposal["value"]
        if value > 1.0:
            return 1.0
        if value < -1.0:
            return -1.0
        return value

