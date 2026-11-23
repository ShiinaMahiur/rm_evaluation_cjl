#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""aim_search.py —— 自瞄演示的「搜索目标」步骤

刚启动仿真器时云台朝向不固定，可能没有任何装甲板在画面里，S3 就永远拿不到观测。
本脚本做一件很朴素的事：缓慢左右扫云台，直到 /solver/armors 出现有效目标，
然后对准最近的那块装甲板，把云台交给 S4 规划器。

它与 S4 的 /rm_gimbal/cmd 不冲突：S4 只有在 S3 有目标时才发布指令，
而本脚本在找到目标后就退出。

用法：
  python3 scripts/aim_search.py             # 默认：从当前朝向开始，左右各扫 180°
  python3 scripts/aim_search.py --span 90   # 只扫左右 90°
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

try:
    from rm_interfaces.msg import GimbalCmd
except Exception:
    GimbalCmd = None


def quat2R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class AimSearch(Node):
    def __init__(self, span_deg):
        super().__init__("aim_search")
        self.span = math.radians(span_deg)
        self.cmd_pub = self.create_publisher(GimbalCmd, "/rm_gimbal/cmd", 10) if GimbalCmd else None
        self.tf = {}
        self.last_armors = []
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)
        self.create_subscription(String, "/solver/armors", self.on_armors, 10)

    def on_tf(self, msg):
        self.tf = {(t.header.frame_id, t.child_frame_id): t.transform for t in msg.transforms}

    def on_armors(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_armors = [a for a in (data.get("armors") or []) if a.get("t_odom")]

    # ---------- 工具 ----------
    def gimbal_pose(self):
        """map→gimbal_link 的位姿（TF 是 map→odom→gimbal_link 两级链，需自己组合）"""
        T = np.eye(4)
        direct = self.tf.get(("map", "gimbal_link"))
        if direct is not None:
            T[:3, :3] = quat2R(direct.rotation.x, direct.rotation.y, direct.rotation.z, direct.rotation.w)
            T[:3, 3] = [direct.translation.x, direct.translation.y, direct.translation.z]
        else:
            for key in (("map", "odom"), ("odom", "gimbal_link")):
                tr = self.tf.get(key)
                if tr is None:
                    return None, None
                M = np.eye(4)
                M[:3, :3] = quat2R(tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)
                M[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
                T = T @ M
        return math.atan2(T[1, 0], T[0, 0]), T[:3, 3].copy()

    def gimbal_yaw(self):
        return self.gimbal_pose()

    def publish_deg(self, yaw_deg, pitch_deg, distance):
        if self.cmd_pub is None:
            return
        cmd = GimbalCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.yaw = float(yaw_deg)
        cmd.pitch = float(pitch_deg)
        cmd.distance = float(distance)
        cmd.fire_advice = False
        self.cmd_pub.publish(cmd)

    def spin(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.02)

    def best_target(self):
        yaw, pos = self.gimbal_pose()
        if yaw is None or not self.last_armors:
            return None
        best = None
        for a in self.last_armors:
            t = np.array(a["t_odom"], dtype=float)
            d = float(np.linalg.norm(t[:2] - pos[:2]))
            if d < 1.0 or d > 12.0:      # 太近是自己车/邻车，太远打不到
                continue
            dyaw = abs(math.atan2(t[1] - pos[1], t[0] - pos[0]) - yaw)
            dyaw = min(dyaw, 2 * math.pi - dyaw)
            score = dyaw + 0.05 * d
            if best is None or score < best[0]:
                best = (score, t, d)
        return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--span", type=float, default=180.0)
    ap.add_argument("--step", type=float, default=15.0)
    ap.add_argument("--settle", type=float, default=0.45)
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()
    rclpy.init()
    node = AimSearch(args.span)
    if node.cmd_pub is None:
        print("[X] 没有 rm_interfaces/GimbalCmd，无法控制云台", file=sys.stderr)
        return
    # 等 TF 起来（刚启动仿真器时可能还没发布）
    yaw0 = None
    t0 = time.time()
    while time.time() - t0 < 10.0:
        node.spin(0.5)
        yaw0, _ = node.gimbal_pose()
        if yaw0 is not None:
            break
    if yaw0 is None:
        print("[X] 拿不到 gimbal_link 的 TF（仿真器在跑吗？）", file=sys.stderr)
        return
    yaw0_deg = math.degrees(yaw0)
    print(f"[搜索] 起始 yaw {yaw0_deg:+.1f}°，左右各扫 {args.span:.0f}°")

    offsets = [0.0]
    k = args.step
    while k <= args.span:
        offsets += [k, -k]
        k += args.step

    t_start = time.time()
    for off in offsets:
        if time.time() - t_start > args.timeout:
            break
        yaw_cmd = yaw0_deg + off
        node.publish_deg(yaw_cmd, 0.0, 5.0)
        node.spin(args.settle)
        best = node.best_target()
        if best is not None:
            _s, t, d = best
            _, pos = node.gimbal_pose()
            yaw_t = math.degrees(math.atan2(t[1] - pos[1], t[0] - pos[0]))
            dz = float(t[2] - pos[2])
            pitch_t = math.degrees(math.atan2(dz, math.hypot(t[0] - pos[0], t[1] - pos[1])))
            print(f"[搜索] 找到目标：距离 {d:.2f} m，转到 yaw {yaw_t:+.1f}° / pitch {pitch_t:+.1f}°")
            for _ in range(10):
                node.publish_deg(yaw_t, pitch_t, d)
                node.spin(0.05)
            print("[搜索] 已对准，交给 S4 规划器")
            node.destroy_node()
            rclpy.shutdown()
            return
    print("[搜索] 扫完了也没找到目标（可能夹角太大/被遮挡）")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
