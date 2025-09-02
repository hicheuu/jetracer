import traitlets

class Racecar(traitlets.HasTraits):
    # 외부에서 값 바꾸면 observe 훅이 호출됨
    steering = traitlets.Float(default_value=0.0)  # -1.0(좌) ~ 0 ~ +1.0(우)
    throttle = traitlets.Float(default_value=0.0)  # -1.0(후진) ~ 0 ~ +1.0(전진)
