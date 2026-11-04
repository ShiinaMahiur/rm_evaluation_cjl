#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""eval_s2_pnp.py —— S2 观测器评估：检测角点 → PnP → 与 /tf 真值对拍

两条曲线（同一脚本）：
  A) 用 **GT 角点** 做 PnP  → 验证解算器本身的几何正确性（理论误差应为 0）
  B) 用 **传统检测器角点** → 端到端（识别误差会传到解算）

指标：
  - 重投影误差（px）
  - 位置误差（m，绝对 + 相对 %）
  - 视线角误差（deg，相机→目标的视线方向差；对应"横向角误差 ≤1°"）
  - 装甲板姿态角误差（deg）
  - 解算成功率（检测框能匹配上 GT 的比例）

用法：
  python3 eval_s2_pnp.py --scenes A B C D E --step 10 --json-out metrics/s2_pnp.json
"""
import argparse
import json
import math
import os
import statistics

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage

from rm_vision_probe.detector import DetectorParams, detect_image
from rm_vision_probe.solver import refine_corners, solve_pnp

PLATE_W, PLATE_H = 0.15, 0.125
# 与检测器输出角点顺序一致：左上→右上→右下→左下（装甲板局部坐标：y=宽 z=高）
OBJ_PTS = np.array([
    [0, -PLATE_W / 2, -PLATE_H / 2],
    [0, +PLATE_W / 2, -PLATE_H / 2],
    [0, +PLATE_W / 2, +PLATE_H / 2],
    [0, -PLATE_W / 2, +PLATE_H / 2],
], dtype=np.float64)


def quat2R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tf_dict(msg):
    return {(t.header.frame_id, t.child_frame_id): t.transform for t in msg.transforms}


def compose(d, chain):
    T = np.eye(4)
    for k in chain:
        tr = d.get(k)
        if tr is None:
            return None
        M = np.eye(4)
        M[:3, :3] = quat2R(tr.rotation)
        M[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
        T = T @ M
    return T


def open_reader(bag_dir):
    style = "NONE"
    try:
        with open(os.path.join(bag_dir, "metadata.yaml"), encoding="utf-8") as fh:
            style = str(yaml.safe_load(fh)["rosbag2_bagfile_information"]
                        .get("compression_mode", "NONE")).upper()
    except Exception:
        pass
    r = (rosbag2_py.SequentialCompressionReader()
         if style in ("FILE", "MESSAGE") else rosbag2_py.SequentialReader())
    r.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    return r


def project_gt(tf, T_map_cam, K, img_w, img_h, min_dist=1.5):
    """返回可见装甲板的 GT: [(name, corners(4,2), t_map, R_map)]"""
    out = []
    Rc, tc = T_map_cam[:3, :3], T_map_cam[:3, 3]
    for (parent, child), tr in tf.items():
        if parent != "map" or not child.startswith("armor_"):
            continue
        Ra = quat2R(tr.rotation)
        ta = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        if np.linalg.norm(ta - tc) < min_dist:
            continue
        if np.dot(Ra @ np.array([1.0, 0, 0]), tc - ta) <= 0:   # 背面
            continue
        pts = []
        for op in OBJ_PTS:
            p_map = ta + Ra @ op
            p_cam = Rc.T @ (p_map - tc)
            if p_cam[2] <= 0.05:
                break
            uv = K @ (p_cam / p_cam[2])
            pts.append(uv[:2])
        if len(pts) < 4:
            continue
        pts = np.array(pts)
        if pts[:, 0].min() < 0 or pts[:, 1].min() < 0 or pts[:, 0].max() > img_w or pts[:, 1].max() > img_h:
            continue
        out.append((child, pts.astype(np.float64), ta, Ra))
    return out


def solve(corners, K):
    return solve_pnp(corners, K)


def rot_angle_deg(A, B):
    d = (np.trace(A.T @ B) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, d))))


def eval_scene(bag_dir, step, params, gt_mode):
    reader = open_reader(bag_dir)
    tf, K = {}, None
    idx = 0
    stats = {"matched": 0, "dets": 0, "solved": 0,
             "rep": [], "pos_err": [], "pos_rel": [], "bearing": [], "rot": []}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == "/tf":
            tf = tf_dict(deserialize_message(raw, TFMessage))
            continue
        if topic == "/camera_info":
            K = np.array(list(deserialize_message(raw, CameraInfo).k), dtype=np.float64).reshape(3, 3)
            continue
        if topic != "/image_raw":
            continue
        idx += 1
        if (idx - 1) % step or not tf or K is None:
            continue
        msg = deserialize_message(raw, Image)
        h, w = msg.height, msg.width
        if gt_mode:
            T_map_cam = compose(tf, [("map", "odom"), ("odom", "gimbal_link"),
                                     ("gimbal_link", "camera_link"), ("camera_link", "camera_optical_frame")])
            if T_map_cam is None:
                continue
            gts = project_gt(tf, T_map_cam, K, w, h)
            for name, corners, ta, Ra in gts:
                stats["solved"] += 1
                r = solve(corners, K)
                if r is None:
                    continue
                R_pnp, t_cam, rep = r
                p_map = T_map_cam[:3, :3] @ t_cam + T_map_cam[:3, 3]
                err = float(np.linalg.norm(p_map - ta))
                stats["rep"].append(rep)
                stats["pos_err"].append(err)
                stats["pos_rel"].append(err / max(np.linalg.norm(ta - T_map_cam[:3, 3]), 1e-6))
                R_gt_cam = T_map_cam[:3, :3].T @ Ra
                stats["rot"].append(rot_angle_deg(R_pnp, R_gt_cam))
        else:
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if buf.size < h * w * 3:
                continue
            img = buf[: h * w * 3].reshape(h, w, 3)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            res = detect_image(img, params)
            T_map_cam = compose(tf, [("map", "odom"), ("odom", "gimbal_link"),
                                     ("gimbal_link", "camera_link"), ("camera_link", "camera_optical_frame")])
            if T_map_cam is None:
                continue
            gts = project_gt(tf, T_map_cam, K, w, h)
            gt_centers = [g[1].mean(axis=0) for g in gts]
            for a in res["armors"]:
                stats["dets"] += 1
                r = solve(refine_corners(a, True), K)
                if r is None:
                    continue
                R_pnp, t_cam, rep = r
                c = refine_corners(a, True).mean(axis=0)
                if not gt_centers:
                    continue
                d = [np.linalg.norm(c - gc) for gc in gt_centers]
                j = int(np.argmin(d))
                if d[j] > 80:
                    continue
                stats["matched"] += 1
                ta = gts[j][2]
                p_map = T_map_cam[:3, :3] @ t_cam + T_map_cam[:3, 3]
                err = float(np.linalg.norm(p_map - ta))
                stats["rep"].append(rep)
                stats["pos_err"].append(err)
                stats["pos_rel"].append(err / max(np.linalg.norm(ta - T_map_cam[:3, 3]), 1e-6))
                v_cam_pnp = t_cam / np.linalg.norm(t_cam)
                v_cam_gt = T_map_cam[:3, :3].T @ (ta - T_map_cam[:3, 3])
                v_cam_gt /= np.linalg.norm(v_cam_gt)
                stats["bearing"].append(math.degrees(math.acos(max(-1.0, min(1.0, float(v_cam_pnp @ v_cam_gt))))))
    def summ(key):
        v = stats[key]
        return (round(statistics.mean(v), 4), round(float(np.percentile(v, 95)), 4)) if v else (None, None)

    return {k: (len(v) if isinstance(v, list) else v) for k, v in stats.items()} | {
        "rep_mean_p95": summ("rep"),
        "pos_err_mean_p95": summ("pos_err"),
        "pos_rel_mean_p95": summ("pos_rel"),
        "bearing_deg_mean_p95": summ("bearing"),
        "rot_deg_mean_p95": summ("rot"),
        "success_rate": round(stats["matched"] / stats["dets"], 4) if stats["dets"] else None,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags-dir", default="/media/chen/RJ/rm_vision/bags")
    ap.add_argument("--scenes", nargs="+", default=["A", "B", "C", "D", "E"])
    ap.add_argument("--step", type=int, default=10)
    ap.add_argument("--json-out", default="metrics/s2_pnp.json")
    args = ap.parse_args()
    params = DetectorParams()
    out = {"plate_size_m": [PLATE_W, PLATE_H], "pnp_flag": "SOLVEPNP_IPPE",
           "results": {}, "traditional": {}}
    for scene in args.scenes:
        bag = os.path.join(args.bags_dir, f"scene{scene}")
        if not os.path.isfile(os.path.join(bag, "metadata.yaml")):
            continue
        print(f"==> scene{scene}: GT 角点 PnP ...", flush=True)
        out["results"][scene] = eval_scene(bag, args.step, params, gt_mode=True)
        print("   ", json.dumps(out["results"][scene], ensure_ascii=False))
        print(f"==> scene{scene}: 传统检测角点 → PnP ...", flush=True)
        out["traditional"][scene] = eval_scene(bag, args.step, params, gt_mode=False)
        print("   ", json.dumps(out["traditional"][scene], ensure_ascii=False))
    os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
    with open(args.json_out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("JSON:", args.json_out)


if __name__ == "__main__":
    main()
