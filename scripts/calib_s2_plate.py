#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""calib_s2_plate.py —— S2 离线标定：仿真 marker ↔ 可见装甲板

解决了什么（详见 README 3.2 / WORKLOG 2026-11-05）
---------------------------------------------------
1. 仿真 `/tf` 的 `armor_*` 是 **marker 立方体中心**（0.03×0.15×0.125，见
   `src/ros2/plugin.rs`），而判分真值应该是**可见装甲板板面中心**；两者在
   marker 局部系里相差 `d ≈ 4 cm`。把 marker 原点直接当板中心，会把这 4 cm
   直接算进「解算误差」——这是之前 A 场景「距离误差 4.44%」的主要来源。
2. 板面在 marker 局部 **x-y 平面**（法向 z），宽度沿 `-e_y`、高度沿 `+e_x`。
3. 传统检测器四角取的是**灯条外沿矩形**，比板外沿窄 4%、矮一半以上；
   需要尺度比 `SX / SY` 才能和 0.15×0.125 的板尺寸对齐。

标定方法
--------
用 GT marker 位姿投影出「板外沿四角」，与检测四角做最小二乘拟合，未知量为
`d`(3) + `SX` + `SY`。目标函数：所有匹配对的四角重投影误差平方和。

输出：`metrics/s2_calib.json`（可直接对照 `solver.py` 里的常数）

用法：
  python3 scripts/calib_s2_plate.py --scenes A B C D E --step 40
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "vision_ws",
                                "src", "rm_vision_probe"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2  # noqa: E402
import eval_s2_pnp as E  # noqa: E402
from rm_vision_probe.detector import DetectorParams, detect_image  # noqa: E402

PLATE_W, PLATE_H = 0.15, 0.125
LOCAL = [(-1, -1), (1, -1), (1, 1), (-1, 1)]      # 左上→右上→右下→左下


def project(T, K, pts_map):
    pc = (T[:3, :3].T @ (pts_map - T[:3, 3]).T).T
    return (K @ (pc / pc[:, 2:3]).T).T[:, :2]


def plate_corners(Ra, ta, d):
    ex, ey = Ra[:, 0], Ra[:, 1]
    base = ta + np.asarray(Ra) @ np.asarray(d)
    return np.array([base + (sx * PLATE_W / 2) * (-ey) + (sy * PLATE_H / 2) * ex
                     for sx, sy in LOCAL])


def expand(q, SX, SY):
    c = q.mean(axis=0)
    return c + (q - c) * np.array([SX, SY])


CYCLIC = [[0, 1, 2, 3], [1, 2, 3, 0], [2, 3, 0, 1], [3, 0, 1, 2]]


def collect_pairs(bags_dir, scenes, step):
    params = DetectorParams()
    pairs = []
    for scene in scenes:
        bag = os.path.join(bags_dir, f"scene{scene}")
        if not os.path.isfile(os.path.join(bag, "metadata.yaml")):
            continue
        reader = E.open_reader(bag)
        tf, K = {}, None
        idx = 0
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            if topic == "/tf":
                tf = E.tf_dict(E.deserialize_message(raw, E.TFMessage))
                continue
            if topic == "/camera_info":
                K = np.array(list(E.deserialize_message(raw, E.CameraInfo).k),
                             dtype=np.float64).reshape(3, 3)
                continue
            if topic != "/image_raw":
                continue
            idx += 1
            if (idx - 1) % step or not tf or K is None:
                continue
            msg = E.deserialize_message(raw, E.Image)
            h, w = msg.height, msg.width
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if buf.size < h * w * 3:
                continue
            img = cv2.cvtColor(buf[: h * w * 3].reshape(h, w, 3), cv2.COLOR_RGB2BGR)
            T = E.compose(tf, [("map", "odom"), ("odom", "gimbal_link"),
                               ("gimbal_link", "camera_link"),
                               ("camera_link", "camera_optical_frame")])
            if T is None:
                continue
            cand = []
            for (parent, child), tr in tf.items():
                if parent != "map" or not child.startswith("armor_"):
                    continue
                ta = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
                if np.linalg.norm(ta - T[:3, 3]) > 6.0:
                    continue
                cand.append((child, E.quat2R(tr.rotation), ta))
            if not cand:
                continue
            res = detect_image(img, params)
            for a in res["armors"]:
                if a.get("subpix") is None:
                    continue
                q = np.asarray(a["subpix"]["corners"], dtype=np.float64)   # 灯条外沿四角
                c = q.mean(axis=0)
                best = (1e9, None)
                for name, Ra, ta in cand:
                    pc = T[:3, :3].T @ (ta - T[:3, 3])
                    if pc[2] <= 0.2:
                        continue
                    uv = (K @ (pc / pc[2]))[:2]
                    dd = float(np.linalg.norm(uv - c))
                    if dd < best[0]:
                        best = (dd, (name, Ra, ta))
                if best[1] is None or best[0] > 100:
                    continue
                pairs.append({"name": best[1][0], "Ra": best[1][1], "ta": best[1][2],
                              "T": T, "K": K, "det": q})
    return pairs


def cost(v, pairs):
    d, SX, SY = v[:3], v[3], v[4]
    tot = 0.0
    for p in pairs:
        gt = project(p["T"], p["K"], plate_corners(p["Ra"], p["ta"], d))
        q = expand(p["det"], SX, SY)
        tot += min(float(np.sum((gt[perm] - q) ** 2)) for perm in CYCLIC)
    return tot


def pattern_search(f, x0, step0, lo, hi, max_iter=8000):
    x = np.array(x0, dtype=float)
    step = np.array(step0, dtype=float)
    best = f(x)
    for _ in range(max_iter):
        improved = False
        for k in range(len(x)):
            for s in (1, -1):
                y = x.copy()
                y[k] += s * step[k]
                if y[k] < lo[k] or y[k] > hi[k]:
                    continue
                c = f(y)
                if c < best - 1e-9:
                    x, best, improved = y, c, True
                    break
        if not improved:
            step *= 0.5
            if step.max() < 1e-6:
                break
    return x, best


def evaluate(pairs, d, SX, SY):
    errs, cerr, hist = [], [], {}
    for p in pairs:
        gt = project(p["T"], p["K"], plate_corners(p["Ra"], p["ta"], d))
        q = expand(p["det"], SX, SY)
        best = (1e9, 0)
        for k, perm in enumerate(CYCLIC):
            e = float(np.mean(np.linalg.norm(gt[perm] - q, axis=1)))
            if e < best[0]:
                best = (e, k)
        errs.append(best[0])
        hist[best[1]] = hist.get(best[1], 0) + 1
        cerr.append(float(np.linalg.norm(gt.mean(axis=0) - q.mean(axis=0))))
    return {"corner_mean_px": round(float(np.mean(errs)), 2),
            "corner_p95_px": round(float(np.percentile(errs, 95)), 2),
            "center_mean_px": round(float(np.mean(cerr)), 2),
            "quad_perm_hist": hist, "n": len(pairs)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags-dir", default="/media/chen/RJ/rm_vision/bags")
    ap.add_argument("--scenes", nargs="+", default=["A", "B", "C", "D", "E"])
    ap.add_argument("--step", type=int, default=40)
    ap.add_argument("--json-out", default="metrics/s2_calib.json")
    args = ap.parse_args()

    pairs = collect_pairs(args.bags_dir, args.scenes, args.step)
    print("匹配对:", len(pairs), flush=True)
    if not pairs:
        return
    out = {"pairs": len(pairs), "scenes": args.scenes, "step": args.step}
    out["before"] = evaluate(pairs, np.zeros(3), 1.0, 1.0)
    lo = [-0.15, -0.15, -0.15, 0.5, 0.5]
    hi = [0.15, 0.15, 0.15, 3.0, 3.0]
    x, best = pattern_search(lambda v: cost(v, pairs),
                             [0.0, 0.0, 0.0, 1.0, 1.0],
                             [0.005, 0.005, 0.005, 0.02, 0.02], lo, hi)
    out["after"] = evaluate(pairs, x[:3], x[3], x[4])
    out["constants"] = {
        "MARKER_TO_PLATE_D": [round(float(v), 5) for v in x[:3]],
        "DET_SCALE_X": round(float(x[3]), 4),
        "DET_SCALE_Y": round(float(x[4]), 4),
        "residual_rms_px": round(float(np.sqrt(best / len(pairs) / 4)), 3),
    }
    print(json.dumps(out, ensure_ascii=False, indent=2))
    os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
    with open(args.json_out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("JSON:", args.json_out)


if __name__ == "__main__":
    main()
