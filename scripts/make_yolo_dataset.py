#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_yolo_dataset.py —— 用仿真真值（/tf）自动生成 YOLO 检测数据集

思路（为什么不需要手工标注）：
  bag 里录了 /tf，其中 `map -> armor_6..17` 是每块装甲板的 marker 位姿。
  把装甲板四角投影到图像平面（相机内参 + 相机外参），就得到精确标注框 —— 零人工标注。

⚠️ 2026-11-05 修正（重要）
  仿真 `/tf` 的 `armor_*` 是 **marker 立方体中心**（0.03×0.15×0.125，见
  `src/ros2/plugin.rs`），板面在 marker 局部 **x-y 平面**，且与可见板面中心相差
  4.24 cm。旧版本按 y-z 平面投影、把 marker 原点当板中心，标注框画在了装甲板
  **底部的地面上**（错的）。现在统一走 `solver.plate_pose_from_marker()`，
  与 S2 解算的真值定义完全一致。

  另外加了「图像证据」过滤：框内必须有灯条亮像素，否则说明这块板被遮挡/背面不可见，
  不生成标注（避免把遮挡物标成装甲板）。

数据划分
  `--val-scenes` 指定验证场景（默认 E，跨场景验证，比随机抽帧更能反映泛化）。
  输出 images/ labels/ debug/ train.txt val.txt dataset.yaml stats.json。

用法：
  python3 make_yolo_dataset.py --scenes A B C D E --step 10 --val-scenes E
"""
import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "vision_ws", "src", "rm_vision_probe"))

from rm_vision_probe.solver import OBJ_PTS, plate_pose_from_marker  # noqa: E402


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


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tf_dict(msg):
    d = {}
    for t in msg.transforms:
        d[(t.header.frame_id, t.child_frame_id)] = t.transform
    return d


def compose(d, chain):
    """按 [(parent,child), ...] 链式合成 4x4 变换"""
    T = np.eye(4)
    for key in chain:
        tr = d.get(key)
        if tr is None:
            return None
        M = np.eye(4)
        M[:3, :3] = quat_to_R(tr.rotation)
        M[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
        T = T @ M
    return T


def project_pt(K, p_cam):
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    x, y, z = p_cam
    if z <= 0.05:
        return None
    return (fx * x / z + cx, fy * y / z + cy)


def lamp_evidence(hsv, box, min_px=4, min_frac=0.004):
    """框内「灯条亮像素」数量与比例 —— 剔除被遮挡/背面不可见的板

    只用亮度/饱和度的粗判据（不依赖检测器），本质是"这块板在图里看不看得见"。
    """
    x0, y0, x1, y1 = [int(round(v)) for v in box]
    x0, y0 = max(x0, 0), max(y0, 0)
    x1, y1 = min(x1, hsv.shape[1]), min(y1, hsv.shape[0])
    if x1 - x0 < 3 or y1 - y0 < 3:
        return False, 0.0
    roi = hsv[y0:y1, x0:x1]
    s, v = roi[:, :, 1], roi[:, :, 2]
    cnt = int(((v > 110) & (s > 90)).sum())
    frac = cnt / float(roi.shape[0] * roi.shape[1])
    return (cnt >= min_px and frac >= min_frac), frac


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags-dir", default="/media/chen/RJ/rm_vision/bags")
    ap.add_argument("--scenes", nargs="+", default=["A", "B", "C", "D", "E"])
    ap.add_argument("--step", type=int, default=10)
    ap.add_argument("--out", default=os.path.expanduser("~/yolo_dataset"))
    ap.add_argument("--min-dist", type=float, default=1.5, help="忽略过近的己方装甲板")
    ap.add_argument("--max-dist", type=float, default=30.0)
    ap.add_argument("--val-scenes", nargs="+", default=["E"], help="留作验证的场景（跨场景验证）")
    ap.add_argument("--debug", type=int, default=6, help="每个场景存几张标注可视化")
    args = ap.parse_args()

    img_dir = os.path.join(args.out, "images")
    lbl_dir = os.path.join(args.out, "labels")
    dbg_dir = os.path.join(args.out, "debug")
    for d in (img_dir, lbl_dir, dbg_dir):
        os.makedirs(d, exist_ok=True)
    for f in glob.glob(os.path.join(args.out, "*.cache")):
        os.remove(f)

    stats, scene_files = {}, {}
    for scene in args.scenes:
        bag = os.path.join(args.bags_dir, f"scene{scene}")
        if not os.path.isfile(os.path.join(bag, "metadata.yaml")):
            print(f"[跳过] 没有 {bag}")
            continue
        reader = open_reader(bag)
        tf_latest = {}
        K = None
        idx = 0
        saved = 0
        dbg_saved = 0
        names = []
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            if topic == "/tf":
                tf_latest = tf_dict(deserialize_message(raw, TFMessage))
                continue
            if topic == "/camera_info":
                ci = deserialize_message(raw, CameraInfo)
                K = list(ci.k)
                continue
            if topic != "/image_raw":
                continue
            idx += 1
            if (idx - 1) % args.step or not tf_latest or K is None:
                continue
            msg = deserialize_message(raw, Image)
            h, w = msg.height, msg.width
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if buf.size < h * w * 3:
                continue
            T_map_cam = compose(tf_latest, [
                ("map", "odom"), ("odom", "gimbal_link"),
                ("gimbal_link", "camera_link"), ("camera_link", "camera_optical_frame")])
            if T_map_cam is None:
                continue
            R_cam = T_map_cam[:3, :3]
            t_cam = T_map_cam[:3, 3]
            hsv = None
            boxes = []
            for (parent, child), tr in tf_latest.items():
                if parent != "map" or not child.startswith("armor_"):
                    continue
                R_m = quat_to_R(tr.rotation)
                t_m = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
                dist = np.linalg.norm(t_m - t_cam)
                if dist < args.min_dist or dist > args.max_dist:
                    continue
                # marker → 真实板面（与 S2 真值定义一致）
                Rp, tp = plate_pose_from_marker(R_m, t_m)
                pts = []
                ok = True
                for op in OBJ_PTS:
                    p_cam = R_cam.T @ (tp + Rp @ op - t_cam)
                    uv = project_pt(K, p_cam)
                    if uv is None:
                        ok = False
                        break
                    pts.append(uv)
                if not ok:
                    continue
                pts = np.array(pts)
                x0, y0 = pts[:, 0].min(), pts[:, 1].min()
                x1, y1 = pts[:, 0].max(), pts[:, 1].max()
                if x1 - x0 < 12 or y1 - y0 < 8:
                    continue
                if x0 < 0 or y0 < 0 or x1 > w or y1 > h:
                    continue
                if hsv is None:
                    hsv = cv2.cvtColor(buf[: h * w * 3].reshape(h, w, 3), cv2.COLOR_RGB2HSV)
                visible, _frac = lamp_evidence(hsv, (x0, y0, x1, y1))
                if not visible:
                    continue
                cx, cy = (x0 + x1) / 2 / w, (y0 + y1) / 2 / h
                bw, bh = (x1 - x0) / w, (y1 - y0) / h
                boxes.append((cx, cy, bw, bh, pts.astype(np.int32)))
            if not boxes:
                continue
            name = f"{scene}_{idx:06d}"
            img = buf[: h * w * 3].reshape(h, w, 3)
            cv2.imwrite(os.path.join(img_dir, name + ".jpg"), cv2.cvtColor(img, cv2.COLOR_RGB2BGR),
                        [cv2.IMWRITE_JPEG_QUALITY, 88])
            with open(os.path.join(lbl_dir, name + ".txt"), "w") as f:
                for (cx, cy, bw, bh, _) in boxes:
                    f.write(f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")
            names.append(name)
            saved += 1
            if dbg_saved < args.debug:
                d = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                for (cx, cy, bw, bh, pts) in boxes:
                    cv2.polylines(d, [pts], True, (0, 255, 0), 2)
                cv2.putText(d, f"{name} boxes={len(boxes)}", (12, 34),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
                cv2.imwrite(os.path.join(dbg_dir, name + ".jpg"), cv2.resize(d, (960, 720)))
                dbg_saved += 1
        stats[scene] = saved
        scene_files[scene] = names
        print(f"scene{scene}: 保存 {saved} 张带标注图片", flush=True)

    # 数据划分：按场景留出（跨场景验证，比随机抽帧更能反映泛化）
    val_scenes = [s for s in args.scenes if s in args.val_scenes]
    train_names, val_names = [], []
    for scene, names in scene_files.items():
        (val_names if scene in val_scenes else train_names).extend(names)
    for tag, names in (("train", train_names), ("val", val_names)):
        with open(os.path.join(args.out, f"{tag}.txt"), "w", encoding="utf-8") as f:
            for n in names:
                f.write(os.path.join(os.path.abspath(img_dir), n + ".jpg") + "\n")
    with open(os.path.join(args.out, "dataset.yaml"), "w", encoding="utf-8") as f:
        yaml.safe_dump({
            "path": os.path.abspath(args.out),
            "train": "train.txt",
            "val": "val.txt",
            "names": {0: "armor"},
            "nc": 1,
        }, f, allow_unicode=True)
    with open(os.path.join(args.out, "stats.json"), "w", encoding="utf-8") as f:
        json.dump({"per_scene": stats, "total": sum(stats.values()),
                   "train": len(train_names), "val": len(val_names),
                   "val_scenes": val_scenes, "step": args.step,
                   "label_source": "tf armor marker -> plate_pose_from_marker (x-y plate plane)"},
                  f, ensure_ascii=False, indent=2)
    print("\n总计:", sum(stats.values()), "张；train:", len(train_names),
          "val:", len(val_names), "；输出:", args.out)


if __name__ == "__main__":
    main()
