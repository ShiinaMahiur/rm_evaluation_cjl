#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gimbal_follow.py —— 回放 bag 时让仿真云台"跟随"录到的云台姿态

原理：
  录 bag 时我们录了 `/gimbal_pose`（仿真里云台的真实位姿，PoseStamped）。
  回放时把它转成 yaw/pitch（**度**）发布到 `/rm_gimbal/cmd`，
  在跑着的 Daedalus 里云台就会跟着复现这段运动。

注意：
  - /rm_gimbal/cmd 的 yaw/pitch 单位是「度」，不是弧度；
  - distance 默认 1.0（避免 -1.0 触发"清空跟踪目标"）；`fire_advice` 恒为 False；
  - 仅用于 S0 回放演示/验收，不参与正式自瞄链路。

用法：
  python3 gimbal_follow.py                       # 默认订阅 /gimbal_pose
  python3 gimbal_follow.py --ros-args -p rate:=60.0
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rm_interfaces.msg import GimbalCmd


def quat_to_yaw_pitch_deg(q) -> tuple[float, float]:
    """四元数 → (yaw, pitch)，单位度。用 YXZ 欧拉角约定。"""
    # 归一化，防止回放数据有数值误差
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) or 1.0
    x, y, z, w = q.x / n, q.y / n, q.z / n, q.w / n
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return math.degrees(yaw), math.degrees(pitch)


class GimbalFollow(Node):
    def __init__(self):
        super().__init__("gimbal_follow")
        # distance 用 1.0 而不是 -1.0：-1.0 在仿真里含义是"无解，清空跟踪目标"
        self.distance = float(self.declare_parameter("distance", 1.0).value)
        self.fire = bool(self.declare_parameter("fire_advice", False).value)
        self.pose = None
        self.sub = self.create_subscription(PoseStamped, "gimbal_pose", self.on_pose, 10)
        self.pub = self.create_publisher(GimbalCmd, "rm_gimbal/cmd", 10)
        self.timer = self.create_timer(1.0 / 100.0, self.tick)  # 100 Hz 发，保证跟得紧
        self.get_logger().info("gimbal_follow 已启动：/gimbal_pose → /rm_gimbal/cmd（单位：度）")

    def on_pose(self, msg: PoseStamped):
        self.pose = msg

    def tick(self):
        if self.pose is None:
            return
        yaw, pitch = quat_to_yaw_pitch_deg(self.pose.pose.orientation)
        cmd = GimbalCmd()
        cmd.yaw = yaw
        cmd.pitch = pitch
        cmd.distance = self.distance
        cmd.fire_advice = self.fire
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = GimbalFollow()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
