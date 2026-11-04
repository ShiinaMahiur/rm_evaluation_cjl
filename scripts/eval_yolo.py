#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""eval_yolo.py —— S1 深度学习路线（YOLOv8）评估：mAP / 检出率 / 耗时对比

产出 metrics/s1_yolo.json（与 s1_traditional.json 同口径，便于对比）。

用法：
  python3 eval_yolo.py --weights ~/yolo_runs/s1_armor/weights/best.pt \
      --val-list ~/yolo_dataset/val.txt --json-out metrics/s1_yolo.json
"""
import argparse
import json
import os
import time

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=os.path.expanduser("~/yolo_runs/s1_armor/weights/best.pt"))
    ap.add_argument("--val-list", default=os.path.expanduser("~/yolo_dataset/val.txt"))
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--json-out", default="metrics/s1_yolo.json")
    ap.add_argument("--latency-samples", type=int, default=100)
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.weights)

    # 1) mAP（用 ultralytics 自带 val）
    print("==> 计算 mAP ...", flush=True)
    m = model.val(data=os.path.expanduser("~/yolo_dataset/dataset.yaml"),
                  imgsz=args.imgsz, conf=0.001, iou=0.6, device=0, verbose=False, plots=False)
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

    ts_gpu, det_gpu = run_latency(0, len(imgs)) if os.environ.get("CUDA_VISIBLE_DEVICES", "0") != "-1" else ([], 0)
    # CPU 延迟（关掉 GPU）
    import torch
    ts_cpu, det_cpu = run_latency("cpu", min(args.latency_samples, len(imgs)))

    out = {
        "detector": "yolo_v8n_autolabel",
        "weights": os.path.abspath(args.weights),
        "imgsz": args.imgsz,
        "conf": args.conf,
        "val_images": len(imgs),
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
        "note": "detection_rate = 至少检出 1 个目标的图片占比（conf>=0.25）；训练数据为仿真真值自动标注",
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
    with open(args.json_out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(json.dumps(out, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
