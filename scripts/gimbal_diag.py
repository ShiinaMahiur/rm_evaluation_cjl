#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gimbal_diag.py —— 云台指令通路诊断（排查 /rm_gimbal/cmd 发了但云台不动）

流程：按一组绝对角度（yaw 扫描、pitch=90 水平）逐条发布，同时读 /gimbal_pose，
打印「发了多少 → 云台实际到多少」。全部不动 = 指令通路有问题。

用法：
  python3 gimbal_diag.py                 # 默认 yaw: 0,30,-30,0（pitch 90）
  python3 gimbal_diag.py --duration 2    # 每个角度保持 2 秒
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
import tf2_ros
from rm_interfaces.msg import GimbalCmd


def quat_to_pitch_yaw(q):
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) or 1.0
    x, y, z, w = q.x / n, q.y / n, q.z / n, q.w / n
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
    return pitch, yaw


class Diag(Node):
    def __init__(self):
        super().__init__("gimbal_diag")
        self.pub = self.create_publisher(GimbalCmd, "/rm_gimbal/cmd", 10)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.cur = None  # odom->gimbal_link 的四元数

    def update(self):
        try:
            t = self.buf.lookup_transform("odom", "gimbal_link", rclpy.time.Time())
            self.cur = t.transform.rotation
        except Exception:
            pass

    @staticmethod
    def rot_delta_deg(a, b):
        if a is None or b is None:
            return 0.0
        d = abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
        d = max(-1.0, min(1.0, d))
        return math.degrees(2.0 * math.acos(d))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=1.5)
    ap.add_argument("--yaws", default="0,30,-30,0")
    args = ap.parse_args()
    rclpy.init()
    node = Diag()
    print("等待 /tf odom->gimbal_link ...")
    t0 = time.monotonic()
    while node.cur is None and time.monotonic() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.2); node.update()
    if node.cur is None:
        print("[X] 15 秒内没拿到 gimbal_link 变换（仿真器没跑？）")
        return
    base = node.cur
    print("初始 gimbal 姿态已记录\n")
    results = []
    for yaw in [float(v) for v in args.yaws.split(",")]:
        cmd = GimbalCmd()
        cmd.yaw, cmd.pitch, cmd.distance, cmd.fire_advice = yaw, 90.0, 1.0, False
        t_end = time.monotonic() + args.duration
        while time.monotonic() < t_end:
            node.pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.02); node.update()
        moved = node.rot_delta_deg(base, node.cur)
        results.append(moved)
        print(f"指令 yaw={yaw:+7.1f}°  →  云台相对初始转了 {moved:6.1f}°  "
              f"({'✅ 有响应' if moved > 10 else '❌ 没动'})")
        time.sleep(0.3)

    print("\n结论：", "✅ 指令通路正常（云台有响应）" if max(results) > 10
          else "❌ 云台完全没响应 → /rm_gimbal/cmd 通路有问题（看仿真日志 [diag] trackers）")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
