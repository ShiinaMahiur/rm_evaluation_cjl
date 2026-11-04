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
from geometry_msgs.msg import PoseStamped
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
        self.create_subscription(PoseStamped, "/gimbal_pose", self.on_pose, 10)
        self.cur = None

    def on_pose(self, msg):
        p, y = quat_to_pitch_yaw(msg.pose.orientation)
        self.cur = (y, p + 90.0)  # (yaw_deg, solver_pitch_deg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=1.5)
    ap.add_argument("--yaws", default="0,30,-30,0")
    args = ap.parse_args()
    rclpy.init()
    node = Diag()
    print("等待 /gimbal_pose ...")
    t0 = time.monotonic()
    while node.cur is None and time.monotonic() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.cur is None:
        print("[X] 15 秒内没收到 /gimbal_pose（仿真器没跑？）")
        return
    print(f"初始云台：yaw={node.cur[0]:+.1f}° pitch={node.cur[1]:.1f}°（90=水平）\n")
    results = []
    for yaw in [float(v) for v in args.yaws.split(",")]:
        cmd = GimbalCmd()
        cmd.yaw, cmd.pitch, cmd.distance, cmd.fire_advice = yaw, 90.0, 1.0, False
        t_end = time.monotonic() + args.duration
        while time.monotonic() < t_end:
            node.pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.02)
        # 停发后再读一次（保持最后一帧状态）
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.02)
        y, p = node.cur if node.cur else (float("nan"), float("nan"))
        results.append((yaw, y))
        print(f"指令 yaw={yaw:+7.1f}°  →  实际 yaw={y:+7.1f}°  "
              f"({'✅ 有响应' if abs(y - yaw) < 15 else '❌ 没动/不到'})")
        time.sleep(0.5)

    moved = any(abs(a - b) > 10 for a, b in results)
    print("\n结论：", "✅ 指令通路正常（云台有响应）" if moved
          else "❌ 云台完全没响应 → /rm_gimbal/cmd 通路有问题（看仿真日志 [diag] trackers）")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
