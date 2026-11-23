#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""eval_s4_planner.py —— S4 规划器离线评估（弹道 + 提前量 + 命中率）

链路（与实机一致）：bag → S1 检测 → S2 PnP → S3 EKF → S4 规划 → 弹道积分 → 命中判定

对比两种弹道补偿方法：
  - approx：θ = atan2(h + g t²/2, d)，t=d/v
  - exact ：解 tanθ 二次方程（低伸解）

指标：
  - 单发命中率：每 1 s 打一发（用当前 EKF 估计 + 提前量），弹道积分后与 GT 装甲板求交
  - 连续命中率：每帧都尝试打一发（受 20 Hz 限制）时的命中比例
  - 平均脱靶距离、平均飞行时间
  - 两种方法的命中率/脱靶距离对比

用法：
  python3 scripts/eval_s4_planner.py --scenes A C --step 5 --json-out metrics/s4_planner.json
"""
import argparse
import json
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, "..", "vision_ws", "src", "rm_vision_probe"))

import cv2  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
import eval_s2_pnp as E  # noqa: E402
from rm_vision_probe.detector import DetectorParams, detect_image  # noqa: E402
from rm_vision_probe.planner import (  # noqa: E402
    plan_aim, plate_hit_test, simulate_trajectory, solve_ballistic)
from rm_vision_probe.solver import (OBJ_PTS, plate_pose_from_marker,  # noqa: E402
                                    refine_corners, solve_pnp)
from rm_vision_probe.tracker import EKF8  # noqa: E402

CYCLIC = [(0, 1, 2, 3), (1, 2, 3, 0), (2, 3, 0, 1), (3, 0, 1, 2)]


def collect(bag_dir, step, params):
    reader = E.open_reader(bag_dir)
    tf, K = {}, None
    idx = 0
    frames = []          # (t, dets{id:{pos,R}}, gt{id:pos}, muzzle(None|pos))
    gt_tl = {}           # id -> [(t, pos, R)]
    muzzle = None
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        t = stamp * 1e-9
        if topic == "/tf":
            tf = E.tf_dict(E.deserialize_message(raw, E.TFMessage))
            for (parent, child), tr in tf.items():
                if parent != "map" or not child.startswith("armor_"):
                    continue
                Rm = E.quat2R(tr.rotation)
                tm = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
                Rp, tp = plate_pose_from_marker(Rm, tm)
                gt_tl.setdefault(child, []).append((t, tp, Rp))
            continue
        if topic == "/camera_info":
            K = np.array(list(E.deserialize_message(raw, E.CameraInfo).k), dtype=float).reshape(3, 3)
            continue
        # 发射点由 /tf 链算（/muzzle_pose 是恒等位姿，不可用）
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
        # muzzle 位置：map→odom→gimbal_link→muzzle
        Tm = np.eye(4)
        for key in (("map", "odom"), ("odom", "gimbal_link"), ("gimbal_link", "muzzle")):
            tr = tf.get(key)
            if tr is None:
                Tm = None
                break
            M = np.eye(4)
            M[:3, :3] = E.quat2R(tr.rotation)
            M[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
            Tm = Tm @ M
        if Tm is None:
            continue
        muzzle = Tm[:3, 3].copy()
        Rc, tc = T[:3, :3], T[:3, 3]
        gts = E.gt_armors(tf, T, K, w, h)
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
            if r is None or r[2] > 6.0:
                continue
            R, tv, _rep = r
            pos_odom = Rc @ tv + tc
            best, best_id = 1e9, None
            for name, gtc, ta, Ra in gts:
                e = min(float(np.mean(np.linalg.norm(gtc[list(p)] - corners, axis=1))) for p in CYCLIC)
                if e < best:
                    best, best_id = e, name
            if best_id is None:
                continue
            gtc = dict((g[0], g[1]) for g in gts)[best_id]
            size = 0.25 * sum(np.linalg.norm(gtc[i] - gtc[(i + 1) % 4]) for i in range(4))
            if best > 0.6 * size:
                continue
            dets[best_id] = {"pos": pos_odom, "R": Rc @ R}
        gt_now = {name: ta for name, _c, ta, _R in gts}
        frames.append((t, dets, gt_now, muzzle.copy()))
    return frames, gt_tl


def interp_gt(gt_tl, aid, t):
    arr = gt_tl.get(aid)
    if not arr:
        return None
    ts = np.array([a[0] for a in arr])
    if t < ts[0] or t > ts[-1]:
        return arr[int(np.argmin(np.abs(ts - t)))][1], arr[int(np.argmin(np.abs(ts - t)))][2]
    i = int(np.searchsorted(ts, t))
    i = max(1, min(i, len(arr) - 1))
    t0, p0, R0 = arr[i - 1]
    t1, p1, R1 = arr[i]
    a = 0.0 if t1 - t0 < 1e-9 else (t - t0) / (t1 - t0)
    return p0 * (1 - a) + p1 * a, R0 if a < 0.5 else R1


def evaluate(frames, gt_tl, method, v=25.0, g=9.81, shot_interval=None):
    counts = {}
    for _t, dets, _gt, _m in frames:
        for aid in dets:
            counts[aid] = counts.get(aid, 0) + 1
    if not counts:
        return None
    target = max(counts, key=counts.get)
    ekf = EKF8(sigma_a=0.2, sigma_alpha=2.756, sigma_pos=0.03, sigma_yaw=0.35)
    shots = hits = 0
    miss, tf_list = [], []
    last_shot = None
    for t, dets, _gt, muzzle in frames:
        d = dets.get(target)
        if d is not None:
            ekf.update_pos(d["pos"], t)
            ekf.update_yaw(math.atan2(d["R"][1, 0], d["R"][0, 0]))
        else:
            ekf.mark_missed(t)
            continue
        if not ekf.initialized or ekf.lost:
            continue
        if last_shot is not None and (t - last_shot) < shot_interval:
            continue
        last_shot = t
        st = ekf.state()
        plan = plan_aim(muzzle, st["pos"], st["vel"], v, g, method=method)
        if plan is None:
            continue
        traj = simulate_trajectory(muzzle, plan["yaw"], plan["pitch"], v, g)
        gp = interp_gt(gt_tl, target, t + plan["t_flight"])
        if gp is None:
            continue
        gpos, gR = gp
        hit, dmin = plate_hit_test(traj, gpos, gR)
        shots += 1
        hits += 1 if hit else 0
        miss.append(dmin)
        tf_list.append(plan["t_flight"])
    if shots == 0:
        return None
    return {
        "method": method,
        "target": target,
        "shots": shots,
        "hits": hits,
        "hit_rate": round(hits / shots, 4),
        "miss_distance_m": {"mean": round(float(np.mean(miss)), 4),
                            "median": round(float(np.median(miss)), 4),
                            "p95": round(float(np.percentile(miss, 95)), 4)},
        "t_flight_ms": {"mean": round(float(np.mean(tf_list)) * 1000.0, 1)},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags-dir", default="/media/chen/RJ/rm_vision/bags")
    ap.add_argument("--scenes", nargs="+", default=["A", "C"])
    ap.add_argument("--step", type=int, default=5)
    ap.add_argument("--json-out", default="metrics/s4_planner.json")
    args = ap.parse_args()
    params = DetectorParams()
    out = {"model": "ballistic exact/approx + lead (fixed-point)", "bullet_speed": 25.0,
           "gravity": 9.81, "scenes": {}}
    for scene in args.scenes:
        bag = os.path.join(args.bags_dir, f"scene{scene}")
        if not os.path.isfile(os.path.join(bag, "metadata.yaml")):
            continue
        print(f"==> 采集 scene{scene} ...", flush=True)
        frames, gt_tl = collect(bag, args.step, params)
        print(f"    帧数 {len(frames)}", flush=True)
        res = {}
        # 单发模式（每秒一发）：测“首发/单发命中率”
        for method in ("exact", "approx"):
            r = evaluate(frames, gt_tl, method, shot_interval=1.0)
            if r:
                res[f"single_{method}"] = r
                print(f"    单发 {method}: hit {r['hit_rate']:.1%} "
                      f"({r['hits']}/{r['shots']}) 脱靶中位 {r['miss_distance_m']['median']} m",
                      flush=True)
        # 连发模式（每帧机会，受 20 Hz 限制）：测连续命中率
        for method in ("exact", "approx"):
            r = evaluate(frames, gt_tl, method, shot_interval=0.05)
            if r:
                res[f"auto_{method}"] = r
                print(f"    连发 {method}: hit {r['hit_rate']:.1%} "
                      f"({r['hits']}/{r['shots']})", flush=True)
        out["scenes"][scene] = res
    os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
    with open(args.json_out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("JSON:", args.json_out)


if __name__ == "__main__":
    main()
