#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gt_target_publisher.py —— 用仿真真值(/tf)驱动 S4 规划器（S4 单独验收用）

为什么需要：S1 传统检测在默认场景（暗光/自车遮挡）里检出率很低，
无法稳定驱动 S2→S3。本脚本直接从 /tf 取某块装甲板的真值位姿/速度，
按 S3 相同格式发布 /tracker/target，从而单独验收 S4（弹道+云台+开火）。

用法：
  python3 scripts/gt_target_publisher.py --armor armor_11
  python3 scripts/gt_target_publisher.py --nearest        # 自动选最近的敌方装甲板
"""
import argparse
import json
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

sys.path.insert(0, "/media/chen/RJ/rm_evaluation_cjl/vision_ws/src/rm_vision_probe")
from rm_vision_probe.solver import plate_pose_from_marker  # noqa: E402


class GTNode(Node):
    def __init__(self, armor, nearest):
        super().__init__("gt_target_publisher")
        self.armor = armor
        self.nearest = nearest
        self.tf = {}
        self.prev = None
        self.pub = self.create_publisher(String, "/tracker/target", 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)
        self.create_timer(0.05, self.on_timer)

    def on_tf(self, msg):
        self.tf = {(t.header.frame_id, t.child_frame_id): t.transform for t in msg.transforms}

    def camera_pos(self):
        T = np.eye(4)
        for key in (("map", "odom"), ("odom", "gimbal_link"),
                    ("gimbal_link", "camera_link"), ("camera_link", "camera_optical_frame")):
            tr = self.tf.get(key)
            if tr is None:
                return None
            M = np.eye(4)
            x, y, z, w = tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w
            M[:3, :3] = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ])
            M[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
            T = T @ M
        return T[:3, 3]

    def plate(self, aid):
        tr = self.tf.get(("map", aid))
        if tr is None:
            return None
        Rm = np.array([[1 - 2 * (tr.rotation.y ** 2 + tr.rotation.z ** 2),
                        2 * (tr.rotation.x * tr.rotation.y - tr.rotation.z * tr.rotation.w),
                        2 * (tr.rotation.x * tr.rotation.z + tr.rotation.y * tr.rotation.w)],
                       [2 * (tr.rotation.x * tr.rotation.y + tr.rotation.z * tr.rotation.w),
                        1 - 2 * (tr.rotation.x ** 2 + tr.rotation.z ** 2),
                        2 * (tr.rotation.y * tr.rotation.z - tr.rotation.x * tr.rotation.w)],
                       [2 * (tr.rotation.x * tr.rotation.z - tr.rotation.y * tr.rotation.w),
                        2 * (tr.rotation.y * tr.rotation.z + tr.rotation.x * tr.rotation.w),
                        1 - 2 * (tr.rotation.x ** 2 + tr.rotation.y ** 2)]])
        tm = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        return plate_pose_from_marker(Rm, tm)

    def on_timer(self):
        cam = self.camera_pos()
        if cam is None:
            return
        targets = []
        for (p, c) in self.tf:
            if p != "map" or not c.startswith("armor_"):
                continue
            r = self.plate(c)
            if r is None:
                continue
            Rp, tp = r
            d = float(np.linalg.norm(tp - cam))
            if d < 1.0:
                continue        # 自己车上
            targets.append((d, c, tp))
        if not targets:
            return
        if self.nearest:
            targets.sort(key=lambda x: x[0])
            d, aid, pos = targets[0]
        else:
            hits = [t for t in targets if t[1] == self.armor]
            if not hits:
                return
            d, aid, pos = hits[0]
        now = self.get_clock().now().nanoseconds * 1e-9
        vel = np.zeros(3)
        if self.prev is not None:
            dt = now - self.prev[0]
            if dt > 1e-3:
                vel = (pos - self.prev[1]) / dt
        self.prev = (now, pos.copy())
        out = {"stamp": {"sec": int(now), "nanosec": int((now - int(now)) * 1e9)},
               "pos": [round(float(v), 4) for v in pos],
               "vel": [round(float(v), 4) for v in vel],
               "yaw": 0.0, "vyaw": 0.0, "lost": False, "initialized": True,
               "color": aid}
        self.pub.publish(String(data=json.dumps(out)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--armor", default=None)
    ap.add_argument("--nearest", action="store_true")
    args = ap.parse_args()
    rclpy.init()
    node = GTNode(args.armor, args.nearest or args.armor is None)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
