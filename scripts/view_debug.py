#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""view_debug.py —— 实时看「检测/瞄准」情况（自瞄调试/录屏用）

开一个窗口，左边是 S1 检测结果（/detector/debug_image，灯条/装甲板/统计都画好了），
并叠加 S4 的瞄准解（/planner/aim：yaw/pitch/距离/是否开火）。

用法（另开一个终端，链路在跑的时候）：
  python3 scripts/view_debug.py
  python3 scripts/view_debug.py --screenshot /tmp/s1_snapshot.jpg   # 按 s 存图
按键：q 退出，s 存一张截图
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

WIN = "自瞄调试：S1 检测 + S4 瞄准（q 退出，s 截图）"


class Viewer(Node):
    def __init__(self):
        super().__init__("view_debug")
        self.frame = None
        self.aim = {}
        self.create_subscription(Image, "/detector/debug_image", self.on_img, qos_profile_sensor_data)
        self.create_subscription(String, "/planner/aim", self.on_aim, 10)

    def on_img(self, msg):
        h, w = msg.height, msg.width
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if msg.encoding == "bgr8":
            img = buf[: h * w * 3].reshape(h, w, 3).copy()   # copy：后面要画字，不能是只读视图
        else:  # rgb8
            img = cv2.cvtColor(buf[: h * w * 3].reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        self.frame = img

    def on_aim(self, msg):
        try:
            self.aim = json.loads(msg.data)
        except json.JSONDecodeError:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--screenshot", default="/tmp/aim_snapshot.jpg")
    ap.add_argument("--scale", type=float, default=0.6)
    args = ap.parse_args()
    rclpy.init()
    node = Viewer()
    show = None
    ok = True
    while ok:
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.frame is None:
            continue
        img = node.frame
        a = node.aim
        if a:
            txt = ("yaw %+.1f°  pitch %+.1f°  d %.2fm  t %.0fms  %s"
                   % (a.get("yaw_deg", 0), a.get("pitch_deg", 0),
                      a.get("horizontal_distance_m", 0), a.get("t_flight_ms", 0),
                      "FIRE" if a.get("fire_advice") else "hold"))
            cv2.putText(img, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 0, 255) if a.get("fire_advice") else (255, 255, 0), 2)
        if args.scale != 1.0:
            disp = cv2.resize(img, None, fx=args.scale, fy=args.scale)
        else:
            disp = img
        cv2.imshow(WIN, disp)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            ok = False
        elif key == ord("s"):
            cv2.imwrite(args.screenshot, disp)
            print("截图已保存:", args.screenshot)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
