#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""eval_s2_pnp.py —— S2 观测器评估：检测角点 → PnP → 与 /tf 真值对拍

两条曲线（同一脚本）：
  A) 用 **GT 角点** 做 PnP  → 验证解算器几何/平面约定是否自洽
  B) 用 **传统检测器角点** → 端到端（识别误差会传到解算）

真值定义（2026-11-05 修正）：
  仿真 /tf 的 `armor_*` 是 marker 立方体中心，板面在 marker 局部 x-y 平面。
  评估用 `solver.plate_pose_from_marker()` 把它换算成「板面中心 + 板面姿态」，
  可用 `--gt marker` 退回旧的「marker 原点」参考点做对照（量化参考点偏移的影响）。

消融开关：
  --no-subpixel   关闭灯条亚像素边缘拟合（退回整像素角点）
  --no-calib      关闭「灯条矩形 → 板外沿」尺度展开（退回原始检测四角）

指标：重投影误差(px)、位置误差(m / 相对%)、视线角误差(deg)、板面姿态角误差(deg)、匹配成功率

用法：
  python3 eval_s2_pnp.py --scenes A B C D E --step 20 --json-out metrics/s2_pnp.json
  python3 eval_s2_pnp.py --scenes A --gt marker --json-out /tmp/x.json
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
from rm_vision_probe.solver import (OBJ_PTS, plate_pose_from_marker,
                                   refine_corners, solve_pnp)


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


def gt_armors(tf, T_map_cam, K, img_w, img_h, gt_ref="plate", min_dist=1.5):
    """返回可见装甲板的真值: [(name, corners(4,2), t_map, R_map)]

    gt_ref="plate" : 板面中心 + 板面姿态（默认，物理正确）
    gt_ref="marker": marker 原点 + marker 姿态（旧定义，只用于对照）
    """
    out = []
    Rc, tc = T_map_cam[:3, :3], T_map_cam[:3, 3]
    for (parent, child), tr in tf.items():
        if parent != "map" or not child.startswith("armor_"):
            continue
        Rm = quat2R(tr.rotation)
        tm = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        if np.linalg.norm(tm - tc) < min_dist:
            continue
        if gt_ref == "plate":
            Ra, ta = plate_pose_from_marker(Rm, tm)
        else:
            Ra, ta = Rm, tm
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


def _summ(v):
    return (round(statistics.mean(v), 4), round(float(np.percentile(v, 95)), 4)) if v else (None, None)


def eval_scene(bag_dir, step, params, mode, gt_ref="plate", subpixel=True, calib=True):
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
        T_map_cam = compose(tf, [("map", "odom"), ("odom", "gimbal_link"),
                                 ("gimbal_link", "camera_link"),
                                 ("camera_link", "camera_optical_frame")])
        if T_map_cam is None:
            continue
        gts = gt_armors(tf, T_map_cam, K, w, h, gt_ref=gt_ref)
        if mode == "gt":
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
            continue
        if not gts:
            continue
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if buf.size < h * w * 3:
            continue
        img = cv2.cvtColor(buf[: h * w * 3].reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        res = detect_image(img, params, subpixel=subpixel)
        # GT 候选：四角 + 投影尺寸（用四角关联，避免斜视/近距离时中心偏移导致漏匹配）
        gt_cands = []
        for name, gtc, ta, Ra in gts:
            size = 0.25 * (np.linalg.norm(gtc[1] - gtc[0]) + np.linalg.norm(gtc[3] - gtc[0])
                           + np.linalg.norm(gtc[2] - gtc[1]) + np.linalg.norm(gtc[2] - gtc[3]))
            gt_cands.append((name, gtc, ta, Ra, size))
        cyc = [(0, 1, 2, 3), (1, 2, 3, 0), (2, 3, 0, 1), (3, 0, 1, 2)]
        for a in res["armors"]:
            stats["dets"] += 1
            corners = refine_corners(a, True, calibrated=calib)
            r = solve(corners, K)
            if r is None:
                continue
            R_pnp, t_cam, rep = r
            best = (1e9, None)   # (误差, GT 候选)
            for cand in gt_cands:
                gtc = cand[1]
                e = min(float(np.mean(np.linalg.norm(gtc[list(p)] - corners, axis=1))) for p in cyc)
                if e < best[0]:
                    best = (e, cand)
            if best[1] is None or best[0] > 0.5 * best[1][4]:
                continue
            stats["matched"] += 1
            name, gtc, ta, Ra, _ = best[1]
            p_map = T_map_cam[:3, :3] @ t_cam + T_map_cam[:3, 3]
            err = float(np.linalg.norm(p_map - ta))
            stats["rep"].append(rep)
            stats["pos_err"].append(err)
            stats["pos_rel"].append(err / max(np.linalg.norm(ta - T_map_cam[:3, 3]), 1e-6))
            v_cam_pnp = t_cam / np.linalg.norm(t_cam)
            v_cam_gt = T_map_cam[:3, :3].T @ (ta - T_map_cam[:3, 3])
            v_cam_gt /= np.linalg.norm(v_cam_gt)
            stats["bearing"].append(math.degrees(math.acos(max(-1.0, min(1.0, float(v_cam_pnp @ v_cam_gt))))))
            R_gt_cam = T_map_cam[:3, :3].T @ Ra
            stats["rot"].append(rot_angle_deg(R_pnp, R_gt_cam))

    out = {k: (len(v) if isinstance(v, list) else v) for k, v in stats.items()}
    out.update({
        "rep_mean_p95": _summ(stats["rep"]),
        "pos_err_mean_p95": _summ(stats["pos_err"]),
        "pos_rel_mean_p95": _summ(stats["pos_rel"]),
        "bearing_deg_mean_p95": _summ(stats["bearing"]),
        "rot_deg_mean_p95": _summ(stats["rot"]),
        "success_rate": round(stats["matched"] / stats["dets"], 4) if stats["dets"] else None,
    })
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags-dir", default="/media/chen/RJ/rm_vision/bags")
    ap.add_argument("--scenes", nargs="+", default=["A", "B", "C", "D", "E"])
    ap.add_argument("--step", type=int, default=20)
    ap.add_argument("--gt", choices=["plate", "marker"], default="plate")
    ap.add_argument("--no-subpixel", action="store_true")
    ap.add_argument("--no-calib", action="store_true")
    ap.add_argument("--json-out", default="metrics/s2_pnp.json")
    args = ap.parse_args()
    params = DetectorParams()
    subpixel = not args.no_subpixel
    calib = not args.no_calib
    out = {"plate_size_m": [0.15, 0.125], "pnp_flag": "SOLVEPNP_IPPE",
           "gt_ref": args.gt, "subpixel": subpixel, "calibrated": calib,
           "step": args.step, "results": {}, "traditional": {}}
    for scene in args.scenes:
        bag = os.path.join(args.bags_dir, f"scene{scene}")
        if not os.path.isfile(os.path.join(bag, "metadata.yaml")):
            continue
        print(f"==> scene{scene}: GT 角点 PnP ...", flush=True)
        out["results"][scene] = eval_scene(bag, args.step, params, "gt",
                                           args.gt, subpixel, calib)
        print("   ", json.dumps(out["results"][scene], ensure_ascii=False))
        print(f"==> scene{scene}: 传统检测角点 → PnP ...", flush=True)
        out["traditional"][scene] = eval_scene(bag, args.step, params, "det",
                                               args.gt, subpixel, calib)
        print("   ", json.dumps(out["traditional"][scene], ensure_ascii=False))
    os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
    with open(args.json_out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("JSON:", args.json_out)


if __name__ == "__main__":
    main()
