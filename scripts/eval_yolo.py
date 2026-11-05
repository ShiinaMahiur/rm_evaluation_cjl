#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""eval_yolo.py —— S1 深度学习路线（YOLOv8）评估：mAP / 检出率 / 耗时对比

产出 metrics/s1_yolo.json（与 s1_traditional.json 同口径，便于对比）。

⚠️ 2026-11-05：数据集标注修正后重训。验证集是**跨场景**的（默认只留 E 作验证，
训练只用 A/B/C/D），因此 mAP 反映的是跨场景泛化，不是同段 bag 相邻帧的"虚高"。

用法：
  python3 scripts/eval_yolo.py --weights ~/yolo_runs/s1_armor_v2/weights/best.pt \
      --val-list ~/yolo_dataset/val.txt --json-out metrics/s1_yolo.json --per-scene
"""
import argparse
import glob
import json
import os
import time

import cv2
import numpy as np


def iou(a, b):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    iw, ih = max(ix1 - ix0, 0.0), max(iy1 - iy0, 0.0)
    inter = iw * ih
    ua = max(ax1 - ax0, 0) * max(ay1 - ay0, 0) + max(bx1 - bx0, 0) * max(by1 - by0, 0) - inter
    return inter / ua if ua > 0 else 0.0


def read_yolo_label(path, w, h):
    out = []
    if not os.path.isfile(path):
        return out
    for ln in open(path, encoding="utf-8"):
        parts = ln.split()
        if len(parts) < 5:
            continue
        _c, cx, cy, bw, bh = [float(x) for x in parts[:5]]
        out.append(((cx - bw / 2) * w, (cy - bh / 2) * h,
                    (cx + bw / 2) * w, (cy + bh / 2) * h))
    return out


def eval_scene(model, img_dir, lbl_dir, scene, imgsz, conf, device):
    files = sorted(glob.glob(os.path.join(img_dir, f"{scene}_*.jpg")))
    if not files:
        return None
    det_imgs = 0
    gts = tps = preds = 0
    for p in files:
        name = os.path.splitext(os.path.basename(p))[0]
        im = cv2.imread(p)
        if im is None:
            continue
        h, w = im.shape[:2]
        gt = read_yolo_label(os.path.join(lbl_dir, name + ".txt"), w, h)
        r = model.predict(p, imgsz=imgsz, conf=conf, device=device, verbose=False)[0]
        boxes = [tuple(float(v) for v in b.xyxy[0]) for b in r.boxes]
        preds += len(boxes)
        gts += len(gt)
        if boxes:
            det_imgs += 1
        used = set()
        for g in gt:
            best, bi = 0.0, -1
            for i, b in enumerate(boxes):
                if i in used:
                    continue
                v = iou(g, b)
                if v > best:
                    best, bi = v, i
            if best >= 0.5:
                tps += 1
                used.add(bi)
    return {"images": len(files), "instances": gts, "pred_boxes": preds,
            "det_images": det_imgs,
            "det_rate": round(det_imgs / len(files), 4),
            "recall": round(tps / max(gts, 1), 4),
            "precision": round(tps / max(preds, 1), 4)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=os.path.expanduser("~/yolo_runs/s1_armor_v2/weights/best.pt"))
    ap.add_argument("--data", default=os.path.expanduser("~/yolo_dataset/dataset.yaml"))
    ap.add_argument("--val-list", default=os.path.expanduser("~/yolo_dataset/val.txt"))
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--json-out", default="metrics/s1_yolo.json")
    ap.add_argument("--latency-samples", type=int, default=100)
    ap.add_argument("--per-scene", action="store_true", help="逐场景统计检出率/召回/精度")
    ap.add_argument("--scenes", nargs="+", default=["A", "B", "C", "D", "E"])
    ap.add_argument("--dataset-dir", default=os.path.expanduser("~/yolo_dataset"))
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.weights)

    # 1) mAP（用 ultralytics 自带 val；验证集按 dataset.yaml 的 val.txt）
    print("==> 计算 mAP ...", flush=True)
    m = model.val(data=args.data, imgsz=args.imgsz, conf=0.001, iou=0.6,
                  device=0, verbose=False, plots=False)
    map50 = float(m.box.map50)
    map5095 = float(m.box.map)
    precision = float(m.box.mp)
    recall = float(m.box.mr)

    with open(args.val_list, encoding="utf-8") as f:
        imgs = [ln.strip() for ln in f if ln.strip()]
    print(f"==> 验证集 {len(imgs)} 张，计算检出率/耗时 ...", flush=True)

    def run_latency(device, n):
        ts = []
        det = 0
        for p in imgs[:n]:
            t0 = time.perf_counter()
            r = model.predict(p, imgsz=args.imgsz, conf=args.conf, device=device, verbose=False)
            ts.append((time.perf_counter() - t0) * 1000.0)
            if len(r[0].boxes) > 0:
                det += 1
        return ts, det

    ts_gpu, det_gpu = ([], 0)
    if os.environ.get("CUDA_VISIBLE_DEVICES", "0") != "-1":
        try:
            ts_gpu, det_gpu = run_latency(0, len(imgs))
        except Exception as e:
            print("GPU 不可用，跳过:", e)
    ts_cpu, det_cpu = run_latency("cpu", min(args.latency_samples, len(imgs)))

    per_scene = {}
    if args.per_scene:
        print("==> 逐场景统计（图像级检出率 / 实例级召回与精度）...", flush=True)
        img_dir = os.path.join(args.dataset_dir, "images")
        lbl_dir = os.path.join(args.dataset_dir, "labels")
        for s in args.scenes:
            r = eval_scene(model, img_dir, lbl_dir, s, args.imgsz, args.conf, 0)
            if r:
                per_scene[s] = r
                print(f"  scene{s}: {json.dumps(r, ensure_ascii=False)}", flush=True)

    out = {
        "detector": "yolo_v8n_autolabel",
        "weights": os.path.abspath(args.weights),
        "imgsz": args.imgsz,
        "conf": args.conf,
        "val_images": len(imgs),
        "val_scenes": ["E"],
        "map50": round(map50, 4),
        "map50_95": round(map5095, 4),
        "precision": round(precision, 4),
        "recall": round(recall, 4),
        "detection_rate": round(det_gpu / len(imgs), 4) if det_gpu else None,
        "latency_ms_gpu_mean": round(float(np.mean(ts_gpu)), 3) if ts_gpu else None,
        "latency_ms_gpu_p95": round(float(np.percentile(ts_gpu, 95)), 3) if ts_gpu else None,
        "latency_ms_cpu_mean": round(float(np.mean(ts_cpu)), 3) if ts_cpu else None,
        "latency_ms_cpu_p95": round(float(np.percentile(ts_cpu, 95)), 3) if ts_cpu else None,
        "cpu_samples": len(ts_cpu),
        "per_scene": per_scene,
        "note": ("labels 由仿真真值自动生成（plate_pose_from_marker，2026-11-05 修正了旧版错误的板面平面）；"
                 "验证集为跨场景 E（训练只用 A/B/C/D）；detection_rate = 至少检出 1 个目标的图片占比（conf>=0.25）"),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
    with open(args.json_out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(json.dumps(out, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
