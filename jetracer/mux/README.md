JetRacer MUX System

이 디렉토리는 JetRacer의 제어 멀티플렉서(MUX) 를 포함합니다.
조이스틱과 UDP 등 여러 입력 소스 중 하나를 선택하여 실제 차량을 제어합니다.

구조 요약

runner.py
시스템 진입점. mux, joystick, udp 프로세스를 실행하고 로그를 관리합니다.

mux.py
제어의 중심.
현재 모드(Joystick / UDP)를 기준으로 입력을 선택하고 실제 차량(PWM)을 제어합니다.

joystick.py
게임패드 입력을 읽어 UDS로 전송합니다.
장치는 자동 감지되며, 스로틀은 로컬에서 계산됩니다.

udp_recv.py
외부 네트워크(예: AI 모델)에서 UDP 명령을 받아 UDS로 전달합니다.

동작 개요

모든 프로세스는 /tmp/jetracer_ctrl.sock (UDS)을 통해 통신합니다.

조이스틱의 Y 버튼으로 Joystick ↔ UDP 모드를 전환합니다.

실제 스로틀 출력은 mux.py에서만 이루어집니다.

설정값 throttle.neutral은
config/nvidia_racecar_config.json에서 필수로 로드됩니다.

