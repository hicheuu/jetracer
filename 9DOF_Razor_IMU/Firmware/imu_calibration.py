import numpy as np

# -----------------------------
# 1) 쿼터니언 유틸 함수
# -----------------------------
def quat_conj(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z], dtype=float)

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=float)

def quat_to_euler_zyx(q):
    w, x, y, z = q

    # roll(x)
    sinr = 2*(w*x + y*z)
    cosr = 1 - 2*(x*x + y*y)
    roll = np.degrees(np.arctan2(sinr, cosr))

    # pitch(y)
    sinp = 2*(w*y - z*x)
    sinp = np.clip(sinp, -1, 1)
    pitch = np.degrees(np.arcsin(sinp))

    # yaw(z)
    siny = 2*(w*z + x*y)
    cosy = 1 - 2*(y*y + z*z)
    yaw = np.degrees(np.arctan2(siny, cosy))

    return roll, pitch, yaw


# -----------------------------
# 2) CSV 로드 (헤더 스킵)
# -----------------------------
filename = "imu_log_20251212_015611.csv"  # 여기에 파일명 넣기

data = np.genfromtxt(filename, delimiter=",", skip_header=1)

quat_cols = (10, 11, 12, 13)

q_raw = data[:, quat_cols]
q_raw /= np.linalg.norm(q_raw, axis=1, keepdims=True)


# -----------------------------
# 3) 초기 500프레임을 기준으로 오프셋 계산
# -----------------------------
N = 500
q_calib = np.mean(q_raw[:N], axis=0)
q_calib /= np.linalg.norm(q_calib)

q_offset = quat_conj(q_calib)

print("q_calib =", q_calib)
print("q_offset =", q_offset)


# -----------------------------
# 4) 보정 적용
# -----------------------------
q_corr = np.array([
    quat_mul(q_offset, q_raw[i])
    for i in range(len(q_raw))
])
q_corr /= np.linalg.norm(q_corr, axis=1, keepdims=True)


# -----------------------------
# 5) Euler 변환
# -----------------------------
rpy_raw = np.array([quat_to_euler_zyx(q) for q in q_raw])
rpy_corr = np.array([quat_to_euler_zyx(q) for q in q_corr])

print("보정 전 RPY 평균:", rpy_raw[:N].mean(axis=0))
print("보정 후 RPY 평균:", rpy_corr[:N].mean(axis=0))
