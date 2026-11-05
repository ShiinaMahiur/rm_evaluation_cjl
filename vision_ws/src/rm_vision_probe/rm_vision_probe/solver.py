"""S2: 观测器（PnP 解算）核心函数 —— 节点与离线评估共用

输入：S1 输出的装甲板 4 角点（左上→右上→右下→左下）
输出：相机系位姿 (R, t)、重投影误差、距离

三个工程细节（答辩可讲）：
  1. **板面坐标系约定**（2026-11-05 修正）：仿真 `/tf` 的 `armor_*` 是 0.03×0.15×0.125
     的 marker 立方体中心，板面在 marker 局部 **x-y 平面**（法向 z），且 marker 原点
     与可见板中心差 `MARKER_TO_PLATE_D ≈ 4 cm`。之前把板面当成 y-z 平面、直接拿
     marker 原点当板中心，是「GT 角点验证误差≈0 但端到端误差 4%」的原因之一：
     平面约定错不影响 PnP 位置（矩形对称），但参考点偏移会直接算进误差。
  2. **检测角点 → 板外沿的尺度标定**：传统法四角取的是灯条外沿矩形（比板外沿窄
     ~4%、矮一半以上），直接按 0.15×0.125 解算会把目标算远。用离线拟合得到的
     `DET_SCALE_X / DET_SCALE_Y` 把灯条矩形展开到板外沿，再解算。
  3. **PnP 方法**：平面目标用 `SOLVEPNP_IPPE`（针对平面目标的解析初值 + 迭代优化），
     比默认 ITERATIVE 在大角度斜视时更稳，且不会出现翻转解。

标定来源：`scripts/calib_s2_plate.py`（拟合场景 A/B 共 184 对检测→真值，角点残差 6.3 px）
"""
import math

import cv2
import numpy as np

PLATE_W = 0.15   # 装甲板宽 (m)
PLATE_H = 0.125  # 装甲板高 (m)
PLATE_ASPECT = PLATE_H / PLATE_W

# 装甲板局部系：x=宽、y=高、z=法向；角点顺序 左上→右上→右下→左下
OBJ_PTS = np.array([
    [-PLATE_W / 2, -PLATE_H / 2, 0.0],
    [+PLATE_W / 2, -PLATE_H / 2, 0.0],
    [+PLATE_W / 2, +PLATE_H / 2, 0.0],
    [-PLATE_W / 2, +PLATE_H / 2, 0.0],
], dtype=np.float64)

# ---- 2026-11-05 离线标定常数（拟合 A/B 场景 184 对，见 WORKLOG）----
# marker 局部系里，marker 原点 → 可见板面中心的偏移 (m)
MARKER_TO_PLATE_D = np.array([-0.02922, 0.02052, -0.02369], dtype=np.float64)
# marker 局部系 → 板面坐标系（宽沿 -e_y，高沿 +e_x，法向沿 +e_z）
MARKER_TO_PLATE_R = np.array([
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
], dtype=np.float64)
# 检测四角（灯条外沿矩形）→ 板外沿的尺度比（宽 / 高）
DET_SCALE_X = 1.0421
DET_SCALE_Y = 2.2508


def plate_pose_from_marker(R_marker, t_marker):
    """仿真 marker 位姿 → 装甲板板面位姿（同一坐标系，如 map）

    R_marker/t_marker 来自 /tf 的 `map -> armor_*`；
    返回 (R_plate, t_plate)，板面上角点用 OBJ_PTS 描述（x=宽, y=高, z=法向）。
    """
    R_m = np.asarray(R_marker, dtype=np.float64)
    t_m = np.asarray(t_marker, dtype=np.float64)
    return R_m @ MARKER_TO_PLATE_R, t_m + R_m @ MARKER_TO_PLATE_D


def refine_corners(det, aspect_fix=True, calibrated=True):
    """把检测角点整理成 PnP 用的 4 点

    det: 检测结果字典（含 corners / lights / subpix）
    优先级：det["subpix"]（检测阶段灯条亚像素边缘）> 整像素 lights > 原始 corners。
    calibrated=True 时按 DET_SCALE_X/Y 把「灯条外沿矩形」展开成「板外沿矩形」，
    使 PnP 的物点与 0.15×0.125 的板尺寸一致；亚像素缺失时自动退化，节点仍可用。
    """
    sp = det.get("subpix")
    if det.get("calibrated"):
        # 检测节点已经输出「标定后的板外沿四角」，直接使用，避免二次展开
        return np.asarray(det["corners"], dtype=np.float64)
    if sp is not None and "corners" in sp:
        q = np.asarray(sp["corners"], dtype=np.float64)
        if calibrated:
            c = q.mean(axis=0)
            q = c + (q - c) * np.array([DET_SCALE_X, DET_SCALE_Y])
        return q
    if det.get("lights") and len(det["lights"]) == 2:
        x1, y1, w1, h1 = det["lights"][0]
        x2, y2, w2, h2 = det["lights"][1]
        q = np.array([[x1, y1], [x2 + w2, y2],
                      [x2 + w2, y2 + h2], [x1, y1 + h1]], dtype=np.float64)
        if calibrated:
            c = q.mean(axis=0)
            q = c + (q - c) * np.array([DET_SCALE_X, DET_SCALE_Y])
        return q
    return np.asarray(det["corners"], dtype=np.float64)


def solve_pnp(corners, K, dist=None):
    """平面 PnP（IPPE）→ (R, t, 重投影误差 px)"""
    corners = np.asarray(corners, dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(OBJ_PTS, corners, K,
                                  None if dist is None else np.asarray(dist, dtype=np.float64),
                                  flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    proj, _ = cv2.projectPoints(OBJ_PTS, rvec, tvec, K, None)
    rep = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - corners, axis=1)))
    return R, tvec.reshape(3), rep


def rot_to_quat_msg(R):
    """旋转矩阵 → 四元数 (x,y,z,w)"""
    m = np.asarray(R, dtype=np.float64)
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return [float(x), float(y), float(z), float(w)]
