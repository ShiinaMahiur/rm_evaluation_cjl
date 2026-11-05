#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""train_yolo.py —— 训练 S1 装甲板检测模型（YOLOv8n，单类 armor）

数据集由 `make_yolo_dataset.py` 从仿真真值自动生成（零人工标注）。
默认 40 epochs / imgsz 640 / batch 16，RTX 5060 上约 3~5 分钟。

用法：
  python3 scripts/train_yolo.py --data ~/yolo_dataset/dataset.yaml --name s1_armor_v2
"""
import argparse
import os


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.expanduser("~/yolo_dataset/dataset.yaml"))
    ap.add_argument("--model", default=os.path.expanduser("~/yolov8n.pt"))
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default="0")
    ap.add_argument("--project", default=os.path.expanduser("~/yolo_runs"))
    ap.add_argument("--name", default="s1_armor_v2")
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.model)
    model.train(data=args.data, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
                device=args.device, project=args.project, name=args.name, exist_ok=True)
    best = os.path.join(args.project, args.name, "weights", "best.pt")
    print("BEST:", best)


if __name__ == "__main__":
    main()
