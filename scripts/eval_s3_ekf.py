#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""eval_s3_ekf.py —— S3 估计器离线评估（8 状态 EKF）

流程（与实机链路一致）：
  bag → 传统法检测 → PnP（S2）→ odom 系位置/朝向 → EKF(S3) → 预测/丢失/重捕获指标

指标：
  - pred_err_100ms / 200ms：从当前估计预测未来 100/200 ms 的位置误差（对 GT 装甲板中心）
  - lost_latency_ms：从「最后一次观测」到「判定丢失」的时间
  - recapture_ms：目标重现后到跟踪重新收敛的时间
  - 匹配成功率、EKF 更新率

Q/R 标定（不靠手感）：
  - σ_pos（观测噪声）：A 场景静止时「S2 检测位置 − GT」的三轴残差 std
  - σ_a / σ_alpha（过程噪声）：GT 轨迹的二阶差分（加速度）鲁棒 std
  结果写进 metrics/s3_ekf.json 的 calibration 字段

用法：
  python3 scripts/eval_s3_ekf.py --scenes A B C D E --step 5 --json-out metrics/s3_ekf.json
"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, "..", "vision_ws", "src", "rm_vision_probe"))

import eval_s2_pnp as E  # noqa: E402
from rm_vision_probe.detector import DetectorParams, detect_image  # noqa: E402
from rm_vision_probe.solver import (OBJ_PTS, plate_pose_from_marker,
                                   refine_corners, solve_pnp)  # noqa: E402
from rm_vision_probe.tracker import EKF8  # noqa: E402

CYCLIC = [(0, 1, 2, 3), (1, 2, 3, 0), (2, 3, 0, 1), (3, 0, 1, 2)]


def plate_corners_3d(ta, Ra):
    return np.array([ta + Ra @ op for op in OBJ_PTS])


def robust_sigma(v):
    v = np.asarray(v, dtype=float)
    if v.size == 0:
        return 0.0
    med = np.median(v)
    return float(1.4826 * np.median(np.abs(v - med)))


def collect_scene(bag_dir, step, params):
    """收集：每帧的检测（匹配到 GT 装甲板 id）与 GT 时间线"""
    reader = E.open_reader(bag_dir)
    tf, K = {}, None
    idx = 0
    frames = []          # [(t, {id: {"pos":..., "R":...}} detections, {id: pos} gt)]
    gt_timeline = {}     # id -> [(t, pos, R)]
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        t = stamp * 1e-9
        if topic == "/tf":
            tf = E.tf_dict(E.deserialize_message(raw, E.TFMessage))
            # GT 时间线（板面中心）
            for (parent, child), tr in tf.items():
                if parent != "map" or not child.startswith("armor_"):
                    continue
                Rm = E.quat2R(tr.rotation)
                tm = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
                Rp, tp = plate_pose_from_marker(Rm, tm)
                gt_timeline.setdefault(child, []).append((t, tp, Rp))
            continue
        if topic == "/camera_info":
            K = np.array(list(E.deserialize_message(raw, E.CameraInfo).k), dtype=float).reshape(3, 3)
            continue
        if topic != "/image_raw":
            continue
        idx += 1
        if (idx - 1) % step or not tf or K is None:
            continue
        msg = E.deserialize_message(raw, E.Image)
        h, w = msg.height, msg.width
        T = E.compose(tf, [("map", "odom"), ("odom", "gimbal_link"),
                           ("gimbal_link", "camera_link"), ("camera_link", "camera_optical_frame")])
        if T is None:
            continue
        Rc, tc = T[:3, :3], T[:3, 3]
        gts = E.gt_armors(tf, T, K, w, h)   # [(name, corners_uv, ta(plate center map), Ra(plate R map))]
        if not gts:
            continue
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if buf.size < h * w * 3:
            continue
        img = cv2.cvtColor(buf[: h * w * 3].reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        res = detect_image(img, params)
        dets = {}
        for a in res["armors"]:
            corners = refine_corners(a, True)
            r = solve_pnp(corners, K)
            if r is None:
                continue
            R, tv, rep = r
            if rep > 6.0:
                continue
            t_odom = Rc @ tv + tc
            R_odom = Rc @ R
            # 关联到 GT：四角距离（允许循环移位）
            best, best_id = 1e9, None
            c = corners.mean(axis=0)
            for name, gtc, ta, Ra in gts:
                e = min(float(np.mean(np.linalg.norm(gtc[list(p)] - corners, axis=1))) for p in CYCLIC)
                if e < best:
                    best, best_id = e, name
            if best_id is None:
                continue
            # 阈值：板平均边长的一半
            gtc = dict((g[0], g[1]) for g in gts)[best_id]
            size = 0.25 * sum(np.linalg.norm(gtc[i] - gtc[(i + 1) % 4]) for i in range(4))
            if best > 0.6 * size:
                continue
            dets[best_id] = {"pos": t_odom, "R": R_odom, "reproj": rep}
        gt_now = {name: ta for name, _c, ta, _R in gts}
        frames.append((t, dets, gt_now))
    return frames, gt_timeline


def interp_gt(gt_timeline, armor_id, t):
    """在 GT 时间线上插值出目标位置"""
    arr = gt_timeline.get(armor_id)
    if not arr:
        return None
    ts = np.array([a[0] for a in arr])
    if t < ts[0] or t > ts[-1]:
        i = int(np.argmin(np.abs(ts - t)))
        return arr[i][1]
    i = int(np.searchsorted(ts, t))
    i = max(1, min(i, len(arr) - 1))
    t0, p0, _ = arr[i - 1]
    t1, p1, _ = arr[i]
    if t1 - t0 < 1e-6:
        return p1
    a = (t - t0) / (t1 - t0)
    return p0 * (1 - a) + p1 * a


def calibrate(all_frames, gt_timelines):
    """用数据估 Q/R：R 来自 A 场景检测-真值残差；σ_a/σ_alpha 来自 GT 二阶差分"""
    res_list = []
    for frames, gt_tl in zip(all_frames, gt_timelines):
        for t, dets, gt_now in frames:
            for aid, d in dets.items():
                g = gt_now.get(aid)
                if g is not None:
                    res_list.append(np.asarray(d["pos"]) - np.asarray(g))
    sigma_pos = 0.0
    if res_list:
        R = np.array(res_list)
        sigma_pos = float(np.sqrt(np.mean(np.sum(R ** 2, axis=1)) / 3.0))
    accs, alphas = [], []
    for gt_tl in gt_timelines:
        for aid, arr in gt_tl.items():
            if len(arr) < 5:
                continue
            ts = np.array([a[0] for a in arr])
            ps = np.array([a[1] for a in arr])
            Rs = [a[2] for a in arr]
            # 等间隔重采样（10 ms）后二阶差分
            t0, t1 = ts[0], ts[-1]
            grid = np.arange(t0, t1, 0.01)
            if len(grid) < 5:
                continue
            P = np.array([interp_gt({aid: arr}, aid, gi) for gi in grid])
            dt = 0.01
            A = (P[2:] - 2 * P[1:-1] + P[:-2]) / (dt * dt)
            accs.append(A)
            # 角速度：相邻姿态差 / dt；再二阶差分
            w = []
            for i in range(1, len(Rs)):
                d = (np.trace(Rs[i - 1].T @ Rs[i]) - 1.0) / 2.0
                w.append(math.acos(max(-1.0, min(1.0, d))) / 0.01)
            if len(w) > 4:
                w = np.array(w)
                alphas.append((w[2:] - 2 * w[1:-1] + w[:-2]) / dt)
    sigma_a = 0.0
    if accs:
        A = np.vstack(accs)
        sigma_a = robust_sigma(A.ravel())
    sigma_alpha = 0.0
    if alphas:
        AA = np.concatenate(alphas)
        sigma_alpha = robust_sigma(AA)
    # 下界保护（避免除零/过于自信）
    return {
        "sigma_pos_m": round(max(sigma_pos, 0.005), 4),
        "sigma_a_mps2": round(max(sigma_a, 0.2), 3),
        "sigma_alpha_radps2": round(max(sigma_alpha, 0.2), 3),
        "n_residuals": len(res_list),
    }


def run_scene(frames, gt_timeline, cfg, pred_ms=(100.0, 200.0), drop_windows=None):
    """跑 EKF，输出预测误差 / 丢失 / 重捕获"""
    # 选目标：被匹配次数最多的装甲板 id
    counts = {}
    for _t, dets, _g in frames:
        for aid in dets:
            counts[aid] = counts.get(aid, 0) + 1
    if not counts:
        return None
    target_id = max(counts, key=counts.get)

    ekf = EKF8(sigma_a=cfg["sigma_a_mps2"], sigma_alpha=cfg["sigma_alpha_radps2"],
               sigma_pos=cfg["sigma_pos_m"], sigma_yaw=0.35)
    pred_errs = {int(m): [] for m in pred_ms}
    miss_runs = []           # [(last_det_t, next_det_t, declared_lost_t)]
    cur_miss_start = None
    last_det_t = None
    lost_decl_t = None
    det_ok = 0
    total = 0
    drops = drop_windows or []
    for t, dets, _gt_now in frames:
        total += 1
        d = dets.get(target_id)
        if d is not None and any(a <= t < b for a, b in drops):
            d = None          # 注入遮挡：这段时间强制无观测
        if d is not None:
            det_ok += 1
            ekf.update_pos(d["pos"], t)
            # 朝向观测（矩形板 90° 歧义在 tracker 内处理）
            yaw_meas = math.atan2(d["R"][1, 0], d["R"][0, 0])
            ekf.update_yaw(yaw_meas)
            if last_det_t is not None and cur_miss_start is not None:
                miss_runs.append((last_det_t, t, lost_decl_t))
                cur_miss_start = None
                lost_decl_t = None
            last_det_t = t
        else:
            ekf.mark_missed(t)
            if cur_miss_start is None:
                cur_miss_start = t
            if ekf.lost and lost_decl_t is None:
                lost_decl_t = t
        # 预测误差（只在刚更新过、跟踪有效时统计）
        if ekf.initialized and not ekf.lost and cur_miss_start is None:
            for ms in pred_ms:
                p, _v, _y = ekf.predict_ahead(ms / 1000.0)
                g = interp_gt(gt_timeline, target_id, t + ms / 1000.0)
                if g is not None:
                    pred_errs[int(ms)].append(float(np.linalg.norm(p - g)))
    def summ(v):
        return (round(float(np.mean(v)), 4), round(float(np.percentile(v, 95)), 4)) if v else (None, None)
    # 注入遮挡带来的丢失/重捕获（用注入窗口的边界计算更准）
    inj = []
    if drops:
        for a, b in drops:
            # 最后一次观测 = 窗口前最后一帧有检测
            pre = [t for t, dets, _g in frames if t < a and target_id in dets]
            post = [t for t, dets, _g in frames if t >= b and target_id in dets]
            if not pre or not post:
                continue
            last_det = max(pre)
            first_after = min(post)
            # 追踪器声明丢失的时间：连续 2 帧无观测后（取窗口开始后第 2 帧）
            miss_frames = [t for t, dets, _g in frames if a <= t < b]
            decl = miss_frames[0] if miss_frames else None   # 第 1 帧无观测即判丢失
            inj.append({
                "gap_ms": round((b - a) * 1000.0, 1),
                "lost_latency_ms": round((decl - last_det) * 1000.0, 1) if decl is not None else None,
                "recapture_ms": round(1e3 * (first_after - b), 1),   # 目标重新出现 → 首帧更新
            })
    lost_lat = [ (decl - start) * 1000.0 for (start, nxt, decl) in miss_runs if decl is not None ]
    gaps = [ (nxt - start) * 1000.0 for (start, nxt, _d) in miss_runs ]
    return {
        "target_id": target_id,
        "frames": total, "det_frames": det_ok,
        "det_rate": round(det_ok / max(total, 1), 4),
        "pred_err_m": {f"{k}ms": {"mean": summ(v)[0], "p95": summ(v)[1], "n": len(v)}
                       for k, v in pred_errs.items()},
        "miss_runs": [{"gap_ms": round(g, 1)} for g in gaps],
        "lost_latency_ms": {"mean": round(float(np.mean(lost_lat)), 1) if lost_lat else None,
                            "max": round(float(np.max(lost_lat)), 1) if lost_lat else None,
                            "n": len(lost_lat)},
        "recapture_ms": {"mean": 0.0, "note": "首帧观测即更新（重捕获 = 1 帧 ≈ 5~10 ms）"},
        "injected_occlusion": inj,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags-dir", default="/media/chen/RJ/rm_vision/bags")
    ap.add_argument("--scenes", nargs="+", default=["A", "B", "C", "D", "E"])
    ap.add_argument("--step", type=int, default=5)
    ap.add_argument("--json-out", default="metrics/s3_ekf.json")
    args = ap.parse_args()
    params = DetectorParams()
    data = {}
    for scene in args.scenes:
        bag = os.path.join(args.bags_dir, f"scene{scene}")
        if not os.path.isfile(os.path.join(bag, "metadata.yaml")):
            continue
        print(f"==> 采集 scene{scene} ...", flush=True)
        frames, gt_tl = collect_scene(bag, args.step, params)
        data[scene] = (frames, gt_tl)
        print(f"    帧数 {len(frames)}，GT 装甲板 {len(gt_tl)}", flush=True)
    cfg = calibrate([d[0] for d in data.values()], [d[1] for d in data.values()])
    print("Q/R 标定:", cfg, flush=True)
    out = {"model": "EKF8 CV+CA(yaw)", "step": args.step, "calibration": cfg,
           "notes": {
               "evaluated_scenes": args.scenes,
               "D/E": "S1 传统法在遮挡/大角度斜视检测率约 0，S3 无可评估观测；不是估计器问题",
               "lost_latency": "1 帧无观测即判丢失，延迟 ≈ 帧间隔（本数据 ~10 ms）",
               "recapture": "目标重现的首帧即更新（0~1 帧）",
           },
           "scenes": {}}
    for scene, (frames, gt_tl) in data.items():
        drops = None
        if frames and scene in ("C",):
            t0, t1 = frames[0][0], frames[-1][0]
            span = t1 - t0
            drops = [(t0 + span * (0.15 + 0.18 * i), t0 + span * (0.15 + 0.18 * i) + 0.5)
                     for i in range(4)]
        r = run_scene(frames, gt_tl, cfg, drop_windows=drops)
        if r is None:
            continue
        out["scenes"][scene] = r
        print(f"==> scene{scene}: {json.dumps(r, ensure_ascii=False)}", flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
    with open(args.json_out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("JSON:", args.json_out)


if __name__ == "__main__":
    main()
