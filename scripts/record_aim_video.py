#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""record_aim_video.py —— 录「自瞄实弹演示」视频（从系统图像流录，不依赖桌面录屏）

画面 = S1 检测结果（灯条/装甲板框，/detector/debug_image）+ S4 瞄准解叠加，
证明整条链路在真实运行（目标被检测→跟踪→瞄准→开火）。

用法（链路在跑的时候）：
  python3 scripts/record_aim_video.py --out media/demo_aim.mp4 --duration 40
"""
import argparse
import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Recorder(Node):
    def __init__(self):
        super().__init__("record_aim_video")
        self.frame = None
        self.aim = {}
        self.n_fire = 0
        self.create_subscription(Image, "/detector/debug_image", self.on_img, qos_profile_sensor_data)
        self.create_subscription(String, "/planner/aim", self.on_aim, 10)

    def on_img(self, msg):
        h, w = msg.height, msg.width
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if msg.encoding == "bgr8":
            img = buf[: h * w * 3].reshape(h, w, 3).copy()
        else:
            img = cv2.cvtColor(buf[: h * w * 3].reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        self.frame = img

    def on_aim(self, msg):
        try:
            a = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if a.get("fire_advice"):
            self.n_fire += 1
        self.aim = a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="media/demo_aim.mp4")
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--fps", type=float, default=12.0)
    ap.add_argument("--width", type=int, default=1280)
    args = ap.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)

    rclpy.init()
    node = Recorder()
    # 等第一帧
    t0 = time.time()
    while node.frame is None and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.frame is None:
        print("[X] 等不到 /detector/debug_image（先起 demo_live.sh --debug-view）")
        return
    h, w = node.frame.shape[:2]
    oh = int(args.width * h / w)
    writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"),
                             args.fps, (args.width, oh))
    print(f"[录] {args.width}x{oh} @ {args.fps}fps → {args.out}（{args.duration:.0f}s）")
    t_start = time.time()
    frames = 0
    while time.time() - t_start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.01)
        if node.frame is None:
            continue
        img = cv2.resize(node.frame, (args.width, oh))
        a = node.aim
        # 顶部黑底条，避免和检测器自带的统计文字重叠
        bar = img[0:100, :].copy()
        cv2.rectangle(bar, (0, 0), (args.width - 1, 99), (0, 0, 0), -1)
        img[0:100, :] = cv2.addWeighted(bar, 0.75, img[0:100, :], 0.25, 0)
        label = "AUTO-AIM  S1 detect -> S2 PnP -> S3 EKF -> S4 fire"
        cv2.putText(img, label, (14, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if a:
            txt = ("yaw %+.1f  pitch %+.1f  dist %.2fm  t_flight %.0fms"
                   % (a.get("yaw_deg", 0), a.get("pitch_deg", 0),
                      a.get("horizontal_distance_m", 0), a.get("t_flight_ms", 0)))
            cv2.putText(img, txt, (14, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            if a.get("fire_advice"):
                cv2.putText(img, "FIRE", (14, oh - 20), cv2.FONT_HERSHEY_SIMPLEX,
                            1.2, (0, 0, 255), 3)
                cv2.rectangle(img, (0, 0), (args.width - 1, oh - 1), (0, 0, 255), 4)
        cv2.putText(img, "shots(fire_advice): %d" % node.n_fire,
                    (args.width - 320, oh - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        writer.write(img)
        frames += 1
        time.sleep(1.0 / args.fps)
    writer.release()
    print(f"[录] 完成：{frames} 帧 → {args.out}")


if __name__ == "__main__":
    main()
