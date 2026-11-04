#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""eval_detector.py —— 在 S0 录制的 bag 上离线评估 S1 传统灯条检测器

输出：
  - 每个 bag 的统计（处理帧数、检出率、平均装甲板数、耗时 p50/p95、理论吞吐）
  - 一个汇总 JSON（默认 metrics/s1_traditional.json）
  - 每个 bag 若干张标注图（默认 metrics/s1_samples/）

用法：
  python3 eval_detector.py --bags-dir /media/chen/RJ/rm_vision/bags \
      --scenes A B C D E --step 5 \
      --json-out /media/chen/RJ/rm_evaluation_cjl/metrics/s1_traditional.json \
      --samples-dir /media/chen/RJ/rm_evaluation_cjl/metrics/s1_samples
"""
import argparse
import glob
import json
import os
import statistics
import sys
import time

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

from rm_vision_probe.detector import DetectorParams, detect_image, draw_debug


def open_reader(bag_dir):
    style = "NONE"
    try:
        import yaml
        with open(os.path.join(bag_dir, "metadata.yaml"), encoding="utf-8") as fh:
            style = str(yaml.safe_load(fh)["rosbag2_bagfile_information"]
                        .get("compression_mode", "NONE")).upper()
    except Exception:
        pass
    reader = (rosbag2_py.SequentialCompressionReader()
              if style in ("FILE", "MESSAGE") else rosbag2_py.SequentialReader())
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    return reader


def img_from_msg(msg):
    h, w = msg.height, msg.width
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    e = msg.encoding.lower()
    if e not in ("rgb8", "bgr8") or buf.size < h * w * 3:
        return None
    img = buf[: h * w * 3].reshape(h, w, 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if e == "rgb8" else img


def eval_bag(bag_dir, params, step, samples_dir, max_samples, scene):
    reader = open_reader(bag_dir)
    frames = 0
    det_frames = 0
    total_armors = 0
    lat = []
    saved = 0
    os.makedirs(samples_dir, exist_ok=True)
    idx = 0
    t_start = time.monotonic()
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic != "/image_raw":
            continue
        idx += 1
        if (idx - 1) % step:
            continue
        msg = deserialize_message(raw, Image)
        img = img_from_msg(msg)
        if img is None:
            continue
        t0 = time.perf_counter()
        res = detect_image(img, params)
        lat.append((time.perf_counter() - t0) * 1000.0)
        frames += 1
        n = len(res["armors"])
        total_armors += n
        if n:
            det_frames += 1
        if saved < max_samples and (frames % max(1, 20 // max(1, max_samples)) == 0):
            dbg = draw_debug(img, res, params)
            scale = 960.0 / dbg.shape[1]
            dbg = cv2.resize(dbg, (960, int(dbg.shape[0] * scale)))
            cv2.putText(dbg, f"{scene} frame{idx} armors={n}", (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imwrite(os.path.join(samples_dir, f"{scene}_f{idx:05d}.jpg"), dbg)
            saved += 1
    wall = time.monotonic() - t_start
    lat_sorted = sorted(lat)

    def pct(p):
        if not lat_sorted:
            return 0.0
        k = min(len(lat_sorted) - 1, int(round((len(lat_sorted) - 1) * p)))
        return lat_sorted[k]

    return {
        "scene": scene,
        "frames_processed": frames,
        "step": step,
        "frames_with_detection": det_frames,
        "detection_rate": round(det_frames / frames, 4) if frames else 0.0,
        "total_armors": total_armors,
        "avg_armors_per_frame": round(total_armors / frames, 3) if frames else 0.0,
        "latency_ms_mean": round(statistics.mean(lat), 3) if lat else 0.0,
        "latency_ms_p50": round(pct(0.50), 3),
        "latency_ms_p95": round(pct(0.95), 3),
        "capacity_fps": round(1000.0 / statistics.mean(lat), 1) if lat else 0.0,
        "eval_wall_seconds": round(wall, 1),
        "samples": saved,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags-dir", default="/media/chen/RJ/rm_vision/bags")
    ap.add_argument("--scenes", nargs="+", default=["A", "B", "C", "D", "E"])
    ap.add_argument("--step", type=int, default=5)
    ap.add_argument("--samples", type=int, default=3)
    ap.add_argument("--json-out", default="metrics/s1_traditional.json")
    ap.add_argument("--samples-dir", default="metrics/s1_samples")
    ap.add_argument("--v-thresh", type=int, default=110)
    ap.add_argument("--s-thresh", type=int, default=90)
    args = ap.parse_args()

    params = DetectorParams(v_thresh=args.v_thresh, s_thresh=args.s_thresh)
    results = []
    for scene in args.scenes:
        bag = os.path.join(args.bags_dir, f"scene{scene}")
        if not os.path.isfile(os.path.join(bag, "metadata.yaml")):
            print(f"[跳过] 没有 {bag}")
            continue
        print(f"==> scene{scene} ...", flush=True)
        r = eval_bag(bag, params, args.step, args.samples_dir, args.samples, scene)
        results.append(r)
        print("    ", json.dumps(r, ensure_ascii=False), flush=True)

    out = {
        "detector": "traditional_lightbar",
        "params": {"v_thresh": params.v_thresh, "s_thresh": params.s_thresh,
                   "min_aspect": params.min_aspect, "gap_ratio": list(params.gap_ratio),
                   "min_digit": params.min_digit},
        "note": "detection_rate = 至少检出 1 块装甲板的帧占比（无人工标注的近似指标）",
        "results": results,
        "summary": {
            "frames_processed": sum(r["frames_processed"] for r in results),
            "avg_detection_rate": round(statistics.mean([r["detection_rate"] for r in results]), 4) if results else 0.0,
            "avg_latency_ms": round(statistics.mean([r["latency_ms_mean"] for r in results]), 3) if results else 0.0,
            "min_capacity_fps": round(min([r["capacity_fps"] for r in results]), 1) if results else 0.0,
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
    with open(args.json_out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("\n汇总:", json.dumps(out["summary"], ensure_ascii=False))
    print("JSON:", args.json_out)


if __name__ == "__main__":
    main()
