#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""plot_s2_error.py —— 生成 S2 验收要求的「误差随距离/角度变化曲线」

方法：用 bag 里 `/tf` 的 armor_<id> 真值角点喂 PnP（排除检测误差，只看解算本身），
把解算位姿与真值比较，画 距离→距离相对误差 与 视线夹角→横向角误差 两条曲线。

输出：media/s2_error_curve.png
"""
import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, "..", "vision_ws", "src", "rm_vision_probe"))
import eval_s2_pnp as E  # noqa: E402
from rm_vision_probe.detector import DetectorParams, detect_image  # noqa: E402
from rm_vision_probe.solver import refine_corners, solve_pnp  # noqa: E402

CYCLIC = [(0, 1, 2, 3), (1, 2, 3, 0), (2, 3, 0, 1), (3, 0, 1, 2)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", default="/media/chen/RJ/rm_vision/bags/sceneA")
    ap.add_argument("--step", type=int, default=20)
    ap.add_argument("--out", default="media/s2_error_curve.png")
    args = ap.parse_args()
    reader = E.open_reader(args.bag)
    tf, K = {}, None
    idx = 0
    dists, pos_rel, bear, angles = [], [], [], []
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic == "/tf":
            tf = E.tf_dict(E.deserialize_message(raw, E.TFMessage))
            continue
        if topic == "/camera_info":
            K = np.array(list(E.deserialize_message(raw, E.CameraInfo).k), dtype=float).reshape(3, 3)
            continue
        if topic != "/image_raw":
            continue
        idx += 1
        if (idx - 1) % args.step or not tf or K is None:
            continue
        msg = E.deserialize_message(raw, E.Image)
        T = E.compose(tf, [("map", "odom"), ("odom", "gimbal_link"),
                           ("gimbal_link", "camera_link"), ("camera_link", "camera_optical_frame")])
        if T is None:
            continue
        cam = T[:3, 3]
        gts = E.gt_armors(tf, T, K, msg.width, msg.height)
        if not gts:
            continue
        import cv2
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = cv2.cvtColor(buf[: msg.height * msg.width * 3].reshape(msg.height, msg.width, 3),
                           cv2.COLOR_RGB2BGR)
        det = detect_image(img, DetectorParams())
        gtc_map = {g[0]: g[1] for g in gts}
        ta_map = {g[0]: g[2] for g in gts}
        Ra_map = {g[0]: g[3] for g in gts}
        for a in det["armors"]:
            corners = refine_corners(a, True)
            # 关联到 GT（四角循环匹配，阈值 = 板平均边长 * 0.6）
            best, best_id = 1e9, None
            for name, gtc in gtc_map.items():
                e = min(float(np.mean(np.linalg.norm(gtc[list(p)] - corners, axis=1))) for p in CYCLIC)
                if e < best:
                    best, best_id = e, name
            if best_id is None:
                continue
            gtc = gtc_map[best_id]
            size = 0.25 * sum(np.linalg.norm(gtc[i] - gtc[(i + 1) % 4]) for i in range(4))
            if best > 0.6 * size:
                continue
            ta, Ra = ta_map[best_id], Ra_map[best_id]
            r = solve_pnp(np.array(corners, dtype=np.float64), K)
            if r is None:
                continue
            R, t, _rep = r
            t_odom = T[:3, :3] @ t + cam
            d = float(np.linalg.norm(t_odom - cam))
            if d < 1.0:
                continue
            rel = float(np.linalg.norm(t_odom - ta) / d)
            # 横向角误差：解算位置相对真值位置在相机系下的横向偏移角
            Rc = T[:3, :3]
            err_cam = Rc.T @ (t_odom - ta)
            lateral = float(np.degrees(np.arctan2(err_cam[0], d)))
            # 视线与板法线夹角（斜视程度）
            los = (ta - cam) / max(np.linalg.norm(ta - cam), 1e-6)
            normal = Ra[:, 2]
            ang = float(np.degrees(np.arccos(min(1.0, abs(float(np.dot(los, normal)))))))
            dists.append(d); pos_rel.append(rel * 100); bear.append(abs(lateral)); angles.append(ang)
    if not dists:
        print("[X] 没采到数据")
        return
    dists = np.array(dists); pos_rel = np.array(pos_rel); bear = np.array(bear); angles = np.array(angles)
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    ax[0].scatter(dists, pos_rel, s=8, alpha=0.5)
    ax[0].axhline(5.0, color="r", ls="--", label="target 5%")
    ax[0].set_xlabel("distance (m)"); ax[0].set_ylabel("relative position error (%)")
    ax[0].set_title("S2 PnP: distance vs position error"); ax[0].legend(); ax[0].grid(alpha=0.3)
    ax[1].scatter(angles, bear, s=8, alpha=0.5)
    ax[1].axhline(1.0, color="r", ls="--", label="target 1 deg")
    ax[1].set_xlabel("view angle vs plate normal (deg)"); ax[1].set_ylabel("lateral angle error (deg)")
    ax[1].set_title("S2 PnP: oblique angle vs angle error"); ax[1].legend(); ax[1].grid(alpha=0.3)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, dpi=120)
    print("曲线已保存:", args.out, "样本数", len(dists),
          "| 距离误差 mean %.2f%% p95 %.2f%% | 横向角误差 mean %.3f° p95 %.3f°"
          % (pos_rel.mean(), np.percentile(pos_rel, 95), bear.mean(), np.percentile(bear, 95)))


if __name__ == "__main__":
    main()
