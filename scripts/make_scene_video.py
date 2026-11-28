#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_scene_video.py —— 把 A~E 场景 bag 做成一段「场景录屏」视频

每段：实际 bag 图像 + S1 检测框叠加 + 场景标签（英文，避免字体问题）。
输出 media/demo_A-E.mp4（720p，10 fps）。

用法：
  python3 scripts/make_scene_video.py --sec 15 --out media/demo_A-E.mp4
"""
import argparse
import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, "..", "vision_ws", "src", "rm_vision_probe"))

import eval_s2_pnp as E  # noqa: E402
from rm_vision_probe.detector import DetectorParams, detect_image, draw_debug  # noqa: E402

SCENE_DESC = {
    "A": "Scene A  static target  (~2-3 m)",
    "B": "Scene B  translating target (~2 m/s)",
    "C": "Scene C  spinning + translating",
    "D": "Scene D  occluded 0.5 s then reappears",
    "E": "Scene E  large oblique angle (>60 deg)",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags-dir", default="/media/chen/RJ/rm_vision/bags")
    ap.add_argument("--scenes", nargs="+", default=["A", "B", "C", "D", "E"])
    ap.add_argument("--sec", type=float, default=15.0, help="每个场景取多少秒素材")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--out", default="media/demo_A-E.mp4")
    args = ap.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    params = DetectorParams()
    writer = None
    size = None
    for scene in args.scenes:
        bag = os.path.join(args.bags_dir, f"scene{scene}")
        if not os.path.isfile(os.path.join(bag, "metadata.yaml")):
            print(f"[跳过] {bag} 不存在")
            continue
        print(f"==> scene{scene} ...", flush=True)
        reader = E.open_reader(bag)
        n_img = 0
        img_k = 0
        step = 25      # bag 约 250 fps，取 10 fps
        t_first = None
        while reader.has_next():
            topic, raw, stamp = reader.read_next()
            if topic != "/image_raw":
                continue
            n_img += 1
            img_k += 1
            if (img_k - 1) % step:
                continue
            if t_first is None:
                t_first = stamp
            if stamp - t_first > args.sec * 1e9:
                break
            msg = E.deserialize_message(raw, E.Image)
            h, w = msg.height, msg.width
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            img = cv2.cvtColor(buf[: h * w * 3].reshape(h, w, 3), cv2.COLOR_RGB2BGR)
            res = detect_image(img, params)
            vis = draw_debug(img, res)
            oh = int(args.width * h / w)
            vis = cv2.resize(vis, (args.width, oh))
            if writer is None:
                writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"),
                                         args.fps, (args.width, oh))
                size = (args.width, oh)
            elif vis.shape[1] != size[0]:
                vis = cv2.resize(vis, size)
            # 顶部黑底条 + 标签（避免与检测器统计文字重叠）
            bar = vis[0:110, :].copy()
            cv2.rectangle(bar, (0, 0), (vis.shape[1] - 1, 109), (0, 0, 0), -1)
            vis[0:110, :] = cv2.addWeighted(bar, 0.75, vis[0:110, :], 0.25, 0)
            cv2.putText(vis, SCENE_DESC.get(scene, f"Scene {scene}"), (14, 78),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(vis, "S1 traditional detector  (boxes = detected armors)",
                        (14, vis.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 0), 1)
            writer.write(vis)
    if writer is not None:
        writer.release()
        print("完成:", args.out)
    else:
        print("[X] 没有读到任何画面")


if __name__ == "__main__":
    main()
