"""S3: 估计器 —— 8 状态 EKF（位置/速度/朝向/角速度）

状态：
    x = [px, py, pz, vx, vy, vz, yaw, vyaw]     （odom/map 系，单位 m / m/s / rad / rad/s）

模型（常速 + 常角速，离散白噪声加速度）：
    p_{k+1} = p_k + v_k dt
    v_{k+1} = v_k
    yaw_{k+1} = yaw_k + vyaw_k dt
    vyaw_{k+1} = vyaw_k
    Q 由「加速度方差 σa²」和「角加速度方差 σα²」生成：
      Q_pos = [[dt^4/4, dt^3/2], [dt^3/2, dt^2]] * σa²  （每个轴）
      Q_yaw = [[dt^4/4, dt^3/2], [dt^3/2, dt^2]] * σα²

观测：
    z_pos = [px, py, pz]            （来自 S2 PnP 的 odom 系位置）
    z_yaw = yaw + k*90°             （装甲板朝向；矩形板有 90° 周期歧义，选最靠近预测的 k）

Q/R 不靠手感：σa / σα 由 GT 轨迹的加速度统计得到；σ_pos 由「S2 检测位置 - GT」残差统计得到
（见 scripts/eval_s3_ekf.py，结果写 metrics/s3_ekf.json）。
"""
import math

import numpy as np

YAW_PERIOD = math.pi / 2.0   # 装甲板 90° 周期


def _cv_q(dt, sigma):
    dt2 = dt * dt
    dt3 = dt2 * dt
    dt4 = dt2 * dt2
    return np.array([[dt4 / 4.0, dt3 / 2.0],
                     [dt3 / 2.0, dt2]]) * (sigma * sigma)


class EKF8:
    def __init__(self, sigma_a=1.0, sigma_alpha=2.0, sigma_pos=0.05, sigma_yaw=0.25):
        self.sigma_a = float(sigma_a)
        self.sigma_alpha = float(sigma_alpha)
        self.r_pos = float(sigma_pos) ** 2
        self.r_yaw = float(sigma_yaw) ** 2
        self.x = np.zeros(8)
        self.P = np.eye(8) * 1e3
        self.initialized = False
        self.last_t = None
        self.miss = 0
        self.lost = True

    # ---------- 模型 ----------
    def F(self, dt):
        F = np.eye(8)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        F[6, 7] = dt
        return F

    def Q(self, dt):
        Q = np.zeros((8, 8))
        qa = _cv_q(dt, self.sigma_a)
        for i in (0, 1, 2):
            Q[i, i] += qa[0, 0]
            Q[i, i + 3] += qa[0, 1]
            Q[i + 3, i] += qa[1, 0]
            Q[i + 3, i + 3] += qa[1, 1]
        qy = _cv_q(dt, self.sigma_alpha)
        Q[6, 6] += qy[0, 0]
        Q[6, 7] += qy[0, 1]
        Q[7, 6] += qy[1, 0]
        Q[7, 7] += qy[1, 1]
        return Q

    # ---------- 操作 ----------
    def reset(self, pos, yaw=None, t=None):
        self.x = np.zeros(8)
        self.x[:3] = pos
        if yaw is not None:
            self.x[6] = yaw
        self.P = np.eye(8)
        self.P[:3, :3] *= self.r_pos * 4.0
        self.P[3:6, 3:6] *= 9.0
        self.P[6, 6] = 0.3 ** 2
        self.P[7, 7] = 3.0 ** 2
        self.last_t = t
        self.initialized = True
        self.miss = 0
        self.lost = False

    def predict(self, t):
        if not self.initialized:
            return
        dt = 0.0 if self.last_t is None else (t - self.last_t)
        if dt <= 0:
            return
        dt = min(dt, 0.5)
        F = self.F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q(dt)
        self.last_t = t

    def _update_linear(self, H, z, R):
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(8)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T
        return y, S

    def update_pos(self, pos, t, gate=1.5):
        """位置观测；返回是否接受"""
        if not self.initialized:
            self.reset(np.asarray(pos, dtype=float), t=t)
            return True
        self.predict(t)
        pred = self.x[:3].copy()
        if np.linalg.norm(np.asarray(pos) - pred) > gate:
            # 跳变超过门限：判定为丢失后重新捕获（重初始化）
            self.reset(np.asarray(pos, dtype=float), t=t)
            return True
        H = np.zeros((3, 8))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        R = np.eye(3) * self.r_pos
        self._update_linear(H, np.asarray(pos, dtype=float), R)
        self.miss = 0
        self.lost = False
        return True

    def update_yaw(self, yaw_meas, t=None):
        """朝向观测：处理 90° 周期歧义（选最靠近预测的等价角）"""
        if not self.initialized:
            return False
        if t is not None:
            self.predict(t)
        pred = self.x[6]
        # 选 k 使 yaw_meas + k*90° 最接近 pred
        k = round((pred - yaw_meas) / YAW_PERIOD)
        z = yaw_meas + k * YAW_PERIOD
        H = np.zeros((1, 8))
        H[0, 6] = 1.0
        self._update_linear(H, np.array([z]), np.array([[self.r_yaw]]))
        return True

    def mark_missed(self, t, lost_after_frames=1):
        """本帧没有观测：预测推进；未观测帧数达到 lost_after_frames 即判 lost

        帧间隔约 5~10 ms，1 帧即判 → 丢失判定延迟约 5~10 ms（考核要求 ≤30 ms）。
        """
        if not self.initialized:
            return
        self.predict(t)
        self.miss += 1
        if self.miss >= lost_after_frames:
            self.lost = True

    # ---------- 输出 ----------
    def state(self):
        return {
            "pos": [float(v) for v in self.x[:3]],
            "vel": [float(v) for v in self.x[3:6]],
            "yaw": float(self.x[6]),
            "vyaw": float(self.x[7]),
            "lost": bool(self.lost),
            "initialized": bool(self.initialized),
        }

    def predict_ahead(self, dt):
        """向前预测 dt 秒（不改状态）"""
        if not self.initialized:
            return None
        x = self.x.copy()
        F = self.F(dt)
        x = F @ x
        return x[:3].copy(), x[3:6].copy(), float(x[6])
