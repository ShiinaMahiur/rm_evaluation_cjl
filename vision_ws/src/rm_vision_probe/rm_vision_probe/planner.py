"""S4: 规划器 —— 弹道解算 + 提前量 + 开火判据（纯函数，节点与离线评估共用）

弹道模型（与仿真一致，见 config.toml [projectile]）：
  初速 v = 25 m/s，重力 g = 9.81 m/s²，无空气阻力
  飞行时间 t 满足：h = d·tanθ − g·d²/(2·v²·cos²θ)，其中 d=水平距离, h=高度差

两种补偿方法（考核要求对比）：
  1. 近似法（constant-gravity approximation）：
        t ≈ d / v           （忽略俯仰对水平速度的投影）
        θ ≈ atan2(h + g·t²/2, d)
     快、闭式；远距离/大高差时偏小（打低）。
  2. 迭代/解析法（exact）：
        把 h = d·tanθ − A·(1+tan²θ)，A = g·d²/(2v²) 看成 tanθ 的二次方程
        A·x² − d·x + (h + A) = 0  →  取较小根（低伸弹道）
     精确无近似误差，代价是一次开方。

提前量：飞行时间 t 与目标位置互相依赖（不动点），用 3~4 次迭代收敛：
    t=0 → p = p0 + v_target·t → θ,t → p → …（收敛判据 |Δt| < 1ms）
"""
import math

import numpy as np

GRAVITY = 9.81
BULLET_SPEED = 25.0


def pitch_approx(d, h, v=BULLET_SPEED, g=GRAVITY):
    """近似法：θ = atan2(h + g t²/2, d)，t = d/v"""
    if d <= 1e-3:
        return math.atan2(h, 1e-3)
    t = d / v
    return math.atan2(h + 0.5 * g * t * t, d)


def pitch_exact(d, h, v=BULLET_SPEED, g=GRAVITY):
    """精确解：解 tanθ 的二次方程，返回较小根（低伸弹道）；无解返回 None"""
    if d <= 1e-3:
        return None
    A = g * d * d / (2.0 * v * v)
    disc = d * d - 4.0 * A * (h + A)
    if disc < 0:
        return None
    x = (d - math.sqrt(disc)) / (2.0 * A)      # 低伸解
    return math.atan(x)


def flight_time(d, theta, v=BULLET_SPEED):
    if abs(math.cos(theta)) < 1e-6:
        return float("inf")
    return d / (v * math.cos(theta))


def solve_ballistic(launch, target, v=BULLET_SPEED, g=GRAVITY, method="exact"):
    """给定发射点与目标点（同一坐标系，竖直轴为 z），返回 (yaw, pitch, t_flight)"""
    launch = np.asarray(launch, dtype=float)
    target = np.asarray(target, dtype=float)
    dvec = target - launch
    d = math.hypot(dvec[0], dvec[1])
    h = float(dvec[2])
    if method == "approx":
        theta = pitch_approx(d, h, v, g)
    else:
        theta = pitch_exact(d, h, v, g)
        if theta is None:
            return None
    yaw = math.atan2(dvec[1], dvec[0])
    t = flight_time(d, theta, v)
    return yaw, theta, t


def plan_aim(launch, target_pos, target_vel=None, v=BULLET_SPEED, g=GRAVITY,
             method="exact", lead_iters=4, dt_eps=1e-3):
    """含提前量的瞄准解（不动点迭代）

    返回 dict：yaw(rad), pitch(rad), t_flight(s), aim_point(预测命中点), method
    """
    launch = np.asarray(launch, dtype=float)
    p = np.asarray(target_pos, dtype=float).copy()
    vel = None if target_vel is None else np.asarray(target_vel, dtype=float)
    t = 0.0
    for _ in range(max(1, lead_iters)):
        sol = solve_ballistic(launch, p, v, g, method)
        if sol is None:
            return None
        yaw, pitch, t_new = sol
        if vel is None:
            t = t_new
            break
        p_new = np.asarray(target_pos, dtype=float) + vel * t_new
        if abs(t_new - t) < dt_eps:
            p, t = p_new, t_new
            break
        p, t = p_new, t_new
    sol = solve_ballistic(launch, p, v, g, method)
    if sol is None:
        return None
    yaw, pitch, t = sol
    return {"yaw": yaw, "pitch": pitch, "t_flight": t,
            "aim_point": [float(x) for x in p], "method": method}


def simulate_trajectory(launch, yaw, pitch, v=BULLET_SPEED, g=GRAVITY,
                        t_max=3.0, dt=0.001):
    """按仿真同样的物理积分出弹道点（含重力，无阻力）"""
    launch = np.asarray(launch, dtype=float)
    dirv = np.array([math.cos(pitch) * math.cos(yaw),
                     math.cos(pitch) * math.sin(yaw),
                     math.sin(pitch)])
    p = launch.copy()
    vel = dirv * v
    pts = [p.copy()]
    t = 0.0
    while t < t_max:
        vel[2] -= g * dt
        p = p + vel * dt
        pts.append(p.copy())
        t += dt
        if p[2] < -1.0:
            break
    return np.array(pts)


def plate_hit_test(traj, plate_center, plate_R, half_w=0.075, half_h=0.0625,
                   margin=0.02):
    """弹道点是否穿过装甲板矩形面（板面为局部 x-y 平面，法向 z）

    返回 (hit, min_dist)  —— min_dist 是弹道点到板面矩形区域的最小距离
    """
    pc = np.asarray(plate_center, dtype=float)
    R = np.asarray(plate_R, dtype=float)
    rel = traj - pc
    local = rel @ R            # 世界 → 板局部（R 的列是板轴）
    # 板面：|x|<=half_w, |y|<=half_h, |z| 小
    inside_xy = (np.abs(local[:, 0]) <= half_w + margin) & \
                (np.abs(local[:, 1]) <= half_h + margin)
    zmin = float(np.min(np.abs(local[inside_xy, 2]))) if np.any(inside_xy) else 1e9
    hit = zmin <= 0.05
    # 最小距离（板面矩形区域）
    dx = np.maximum(np.abs(local[:, 0]) - half_w, 0.0)
    dy = np.maximum(np.abs(local[:, 1]) - half_h, 0.0)
    dz = np.abs(local[:, 2])
    dist = np.sqrt(dx * dx + dy * dy + dz * dz)
    return bool(hit), float(np.min(dist))
