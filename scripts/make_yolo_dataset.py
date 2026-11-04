#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_yolo_dataset.py —— 用仿真真值（/tf）自动生成 YOLO 检测数据集

思路（为什么不需要手工标注）：
  bag 里录了 /tf，其中 `map -> armor_6..17` 是每块装甲板的**真值位姿**。
  装甲板尺寸由仿真 marker 定义：宽 0.15 m、高 0.125 m、厚 0.03 m。
  把装甲板四角投影到图像平面（用 /camera_info 的 K + 相机外参），就得到
  精确标注框 —— 不花一分钟人工标注。

输出（YOLO 格式，单类 "armor"）：
  <out>/images/*.jpg        原图
  <out>/labels/*.txt        class cx cy w h（归一化）
  <out>/debug/*.jpg         标注可视化（抽样）
  <out>/dataset.yaml        ultralytics 训练配置

用法：
  python3 make_yolo_dataset.py --scenes A B C D E --step 10
"""
import argparse
import glob
import json
import os

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage


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


def project(K, p_cam):
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    x, y, z = p_cam
    if z <= 0.05:
        return None
    return (fx * x / z + cx, fy * y / z + cy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags-dir", default="/media/chen/RJ/rm_vision/bags")
    ap.add_argument("--scenes", nargs="+", default=["A", "B", "C", "D", "E"])
    ap.add_argument("--step", type=int, default=10)
    ap.add_argument("--out", default=os.path.expanduser("~/yolo_dataset"))
    ap.add_argument("--min-dist", type=float, default=1.5, help="忽略过近的己方装甲板")
    ap.add_argument("--max-dist", type=float, default=30.0)
    ap.add_argument("--plate-w", type=float, default=0.15, help="装甲板宽 (m)，来自仿真 marker")
    ap.add_argument("--plate-h", type=float, default=0.125, help="装甲板高 (m)")
    ap.add_argument("--debug", type=int, default=6, help="每个场景存几张标注可视化")
    args = ap.parse_args()

    img_dir = os.path.join(args.out, "images")
    lbl_dir = os.path.join(args.out, "labels")
    dbg_dir = os.path.join(args.out, "debug")
    for d in (img_dir, lbl_dir, dbg_dir):
        os.makedirs(d, exist_ok=True)

    stats = {}
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
            boxes = []
            for (parent, child), tr in tf_latest.items():
                if parent != "map" or not child.startswith("armor_"):
                    continue
                R_a = quat_to_R(tr.rotation)
                t_a = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
                # 过滤己方/过近的装甲板
                dist = np.linalg.norm(t_a - t_cam)
                if dist < args.min_dist or dist > args.max_dist:
                    continue
                normal = R_a @ np.array([1.0, 0.0, 0.0])   # 装甲板法线（marker x 轴）
                if np.dot(normal, t_cam - t_a) <= 0:        # 背面不可见
                    continue
                hw, hh = args.plate_w / 2, args.plate_h / 2
                pts = []
                ok = True
                for sy, sz in [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]:
                    p_map = t_a + R_a @ np.array([0.0, sy, sz])
                    p_cam = R_cam.T @ (p_map - t_cam)
                    uv = project(K, p_cam)
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
            saved += 1
            if dbg_saved < args.debug:
                d = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                for (cx, cy, bw, bh, pts) in boxes:
                    cv2.polylines(d, [pts], True, (0, 255, 0), 3)
                cv2.putText(d, f"{name} boxes={len(boxes)}", (12, 34),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
                cv2.imwrite(os.path.join(dbg_dir, name + ".jpg"), cv2.resize(d, (960, 720)))
                dbg_saved += 1
        stats[scene] = saved
        print(f"scene{scene}: 保存 {saved} 张带标注图片", flush=True)

    with open(os.path.join(args.out, "dataset.yaml"), "w", encoding="utf-8") as f:
        yaml.safe_dump({
            "path": os.path.abspath(args.out),
            "train": "images",
            "val": "images",
            "names": {0: "armor"},
            "nc": 1,
        }, f, allow_unicode=True)
    with open(os.path.join(args.out, "stats.json"), "w", encoding="utf-8") as f:
        json.dump({"per_scene": stats, "total": sum(stats.values()),
                   "plate_size": [args.plate_w, args.plate_h], "step": args.step}, f,
                  ensure_ascii=False, indent=2)
    print("\n总计:", sum(stats.values()), "张；输出:", args.out)


if __name__ == "__main__":
    main()
