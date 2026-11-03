#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bag_view.py —— 回放 bag 并直接显示图像窗口（不是视频文件，bag 只是话题记录）

为什么需要它：
  `ros2 bag play` 只是把录下来的话题重新"播"出来，**本身不出画面**；
  想"看回放"必须再开一个图像查看器（rqt_image_view / 本脚本的 cv2 窗口）。

用法：
  python3 bag_view.py /media/chen/RJ/rm_vision/bags/sceneB
  python3 bag_view.py <bag> --rate 0.5          # 半速
  python3 bag_view.py <bag> --every 2           # 每 2 帧显示 1 帧（省 CPU）
  窗口里按 q / ESC 退出，会同时停掉回放进程。
"""
import argparse
import subprocess
import sys
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def to_bgr(msg: Image):
    h, w = msg.height, msg.width
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    e = msg.encoding.lower()
    if e in ("rgb8", "bgr8"):
        n = h * w * 3
        if buf.size < n:
            return None
        arr = buf[:n].reshape(h, w, 3)
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if e == "rgb8" else arr
    if e in ("mono8", "8uc1"):
        n = h * w
        return cv2.cvtColor(buf[:n].reshape(h, w), cv2.COLOR_GRAY2BGR) if buf.size >= n else None
    return None


class Viewer(Node):
    def __init__(self, every: int):
        super().__init__("bag_view")
        self.every = max(1, every)
        self.count = 0
        self.frames = 0
        self.create_subscription(Image, "/image_raw", self.on_img, 10)

    def on_img(self, msg: Image):
        self.count += 1
        if self.count % self.every:
            return
        img = to_bgr(msg)
        if img is None:
            return
        self.frames += 1
        cv2.putText(img, f"{self.frames}  {msg.width}x{msg.height} {msg.encoding}",
                    (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.imshow("bag replay  (q / ESC to quit)", img)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--every", type=int, default=1, help="每 N 帧显示 1 帧")
    args = ap.parse_args()

    # 起回放子进程（队列加大，减少 "Message queue starved"）
    play = subprocess.Popen(
        ["ros2", "bag", "play", args.bag, "-r", str(args.rate), "--read-ahead-queue-size", "2000"],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
    )
    rclpy.init()
    node = Viewer(args.every)
    print(f"回放+显示：{args.bag}（rate={args.rate}）—— 图像窗口里按 q 退出")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):  # noqa: BLE001
        pass
    finally:
        play.terminate()
        try:
            play.wait(timeout=3)
        except Exception:  # noqa: BLE001
            play.kill()
        cv2.destroyAllWindows()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    print(f"共显示 {node.frames} 帧")


if __name__ == "__main__":
    main()
