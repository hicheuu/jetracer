IMU 측정을 위한 순서

- imu_reader.py

    UART로 들어오는 accel/gyro/mag/Quat 데이터를 실시간 출력
    이유는 모르겠지만 heading은 제대로 안됨 ㅠㅠ

- calibration_soft_hard.py

    hard-iron 보정 가능

- calibrated_mag_test.py

    실시간 hard-iron 보정정도를 체크가 가능함 

- imu_tilt_compensation.py

    저역통과 필터를 사용해 

##########################################################################

다 필요없고 자이로 센서로만 yaw 측정할거임 드리프트 발생하는건 명석이 인지랑 센서퓨전해서 해결하기!!!

imu 캘리브레이션 하는법 

1. imu_reader.py -> csv 파일 추출
2. imu_clibartion.py -> 추출한 csv 파일로 offset 추출
3. imu_tilt_compensation.py -> imu_clibartion.py 에서 추출된 offset을 직접 적어서 해결 