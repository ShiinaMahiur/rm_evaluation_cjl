#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""follow_target.py —— S0 演示用：图像重心 + P 控制，让云台跟随画面里的目标

作用（S0 验收 "一键回放并让云台跟随"）：
  订阅 bag 回放出来的 /image_raw → 找画面里最亮的彩色块（装甲板/灯光）→
  算它离画面中心多远 → 用 P 控制更新 yaw/pitch 目标 → 发 /rm_gimbal/cmd
  → 仿真器云台跟着转，目标逐渐回到画面中心。

注意：
  - 需要仿真器以 **默认模式**（ROS 外部控制）运行，不要加 DAEDALUS_MANUAL_AIM=1
  - /rm_gimbal/cmd 的 yaw/pitch 单位是**度**；pitch=90 是源码约定的"水平"，
    但不同机体枪口安装偏置不同（Daedalus 实测枪口相对云台约有 -65° 俯仰偏置），
    所以提供 --pitch-base 用来标定基准值（默认 90）
  - 没找到目标时发 distance = -1.0（仿真会清空跟踪目标）

用法：
  python3 follow_target.py                 # 默认参数
  python3 follow_target.py --kp-yaw 0.04 --invert-pitch
  python3 follow_target.py --debug-image   # 另发一张标注图到 /image_probe
"""
import argparse
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from rm_interfaces.msg import GimbalCmd


def quat_to_pitch_yaw(q):
    """四元数 → (pitch, yaw) 弧度（YXZ 欧拉）"""
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) or 1.0
    x, y, z, w = q.x / n, q.y / n, q.z / n, q.w / n
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return pitch, yaw


class FollowTarget(Node):
    def __init__(self, a):
        super().__init__("follow_target")
        self.a = a
        self.pub = self.create_publisher(GimbalCmd, "/rm_gimbal/cmd", 10)
        self.sub = self.create_subscription(Image, self.a.topic, self.on_image, qos_profile_sensor_data)
        self.sub_pose = self.create_subscription(PoseStamped, "/gimbal_pose", self.on_pose, 10)
        self.debug_pub = None
        if a.debug_image:
            self.debug_pub = self.create_publisher(Image, "/image_probe", 10)
        self.yaw_deg = None    # solver 坐标系 yaw（度）
        self.pitch_deg = None  # solver 坐标系 pitch（度，90=水平）
        self.frames = 0
        self.found = 0
        self.t_last = time.monotonic()
        self.get_logger().info(
            f"follow_target 启动 kp=({a.kp_yaw},{a.kp_pitch}) deadzone={a.deadzone}px "
            f"invert=({a.invert_yaw},{a.invert_pitch})"
        )

    def on_pose(self, msg: PoseStamped):
        pitch, yaw = quat_to_pitch_yaw(msg.pose.orientation)
        if self.yaw_deg is None:
            self.yaw_deg = math.degrees(yaw)
            self.pitch_deg = math.degrees(pitch) + self.a.pitch_base

    def target_mask(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        base = (v > self.a.v_thresh) & (s > self.a.s_thresh)
        mask = None
        if self.a.color == "blue":
            blue = base & (h > 90) & (h < 140)
            if int(blue.sum()) > 40:
                mask = blue.astype(np.uint8) * 255
        elif self.a.color == "red":
            red = base & ((h < 12) | (h > 168))
            if int(red.sum()) > 40:
                mask = red.astype(np.uint8) * 255
        if mask is None:
            mask = base.astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        return mask

    def on_image(self, msg: Image):
        if self.yaw_deg is None:
            return
        h, w = msg.height, msg.width
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        e = msg.encoding.lower()
        if e not in ("rgb8", "bgr8") or buf.size < h * w * 3:
            return
        bgr = buf[: h * w * 3].reshape(h, w, 3)
        if e == "rgb8":
            bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)
        # 降采样加速
        scale = self.a.downscale
        small = cv2.resize(bgr, (w // scale, h // scale), interpolation=cv2.INTER_NEAREST)
        mask = self.target_mask(small)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 只保留"紧凑"的块：装甲板灯条是短粗条，场地图案是细长线 —— 用边长/长宽比过滤
        big, area = None, 0.0
        for c in contours:
            a = cv2.contourArea(c)
            if a < self.a.min_area or a > self.a.max_area:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            if bw < self.a.min_side or bh < self.a.min_side:
                continue
            if max(bw, bh) / max(1, min(bw, bh)) > self.a.max_aspect:
                continue
            if a > area:
                big, area = c, a
        now = time.monotonic()
        dt = max(1e-3, now - self.t_last)
        self.t_last = now
        self.frames += 1

        if big is None or area < self.a.min_area:
            # 没找到目标：发 -1 让仿真清空目标；同时缓慢扫 yaw 主动搜索
            if self.a.search_rate != 0.0:
                self.yaw_deg += self.a.search_rate * dt
                if self.yaw_deg > 180.0:
                    self.yaw_deg -= 360.0
            cmd = GimbalCmd()
            cmd.yaw = float(self.yaw_deg)
            cmd.pitch = float(self.pitch_deg)
            cmd.distance = -1.0
            cmd.fire_advice = False
            self.pub.publish(cmd)
            return

        self.found += 1
        m = cv2.moments(big)
        cx = m["m10"] / max(m["m00"], 1e-6) * scale
        cy = m["m01"] / max(m["m00"], 1e-6) * scale
        ex = cx - w / 2.0
        ey = cy - h / 2.0
        if abs(ex) < self.a.deadzone:
            ex = 0.0
        if abs(ey) < self.a.deadzone:
            ey = 0.0
        sx = -1.0 if self.a.invert_yaw else 1.0
        sy = -1.0 if self.a.invert_pitch else 1.0
        if self.a.mode == "absolute":
            # 回放 bag 用：画面不动，像素偏移直接换算成绝对角度（开环映射）
            self.yaw_deg = self.a.yaw_center + sx * (ex / max(w, 1)) * self.a.fov_deg
            self.pitch_deg = self.a.pitch_base + sy * (ey / max(h, 1)) * self.a.fov_deg
        else:
            # P 控制：像素误差 → 角度增量（度/帧），并限速
            self.yaw_deg += sx * self.a.kp_yaw * ex * dt * 60.0
            self.pitch_deg += sy * self.a.kp_pitch * ey * dt * 60.0
        self.yaw_deg = max(-180.0, min(180.0, self.yaw_deg))
        self.pitch_deg = max(self.a.pitch_base - 45.0, min(self.a.pitch_base + 45.0, self.pitch_deg))

        cmd = GimbalCmd()
        cmd.yaw = float(self.yaw_deg)
        cmd.pitch = float(self.pitch_deg)
        cmd.distance = float(self.a.distance)
        cmd.fire_advice = False
        self.pub.publish(cmd)

        if self.debug_pub is not None:
            dbg = bgr.copy()
            cv2.drawContours(dbg, [c for c in contours if c is big], -1, (0, 255, 0), 3)
            cv2.circle(dbg, (int(cx), int(cy)), 8, (0, 0, 255), -1)
            cv2.putText(dbg, f"err=({ex:+.0f},{ey:+.0f}) yaw={self.yaw_deg:+.1f} pitch={self.pitch_deg:.1f}",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            out = Image()
            out.header = msg.header
            out.height, out.width = dbg.shape[:2]
            out.encoding = "bgr8"
            out.is_bigendian = 0
            out.step = out.width * 3
            out.data = dbg.tobytes()
            self.debug_pub.publish(out)

        if self.frames % 30 == 0:
            self.get_logger().info(
                f"frames={self.frames} found={self.found} err=({ex:+.0f},{ey:+.0f}) "
                f"cmd yaw={self.yaw_deg:+.1f} pitch={self.pitch_deg:.1f} area={area:.0f}"
            )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kp-yaw", type=float, default=0.02)
    ap.add_argument("--kp-pitch", type=float, default=0.02)
    ap.add_argument("--deadzone", type=float, default=30.0, help="像素死区")
    ap.add_argument("--invert-yaw", action="store_true")
    ap.add_argument("--invert-pitch", action="store_true")
    ap.add_argument("--color", choices=["any", "blue", "red"], default="blue",
                    help="优先找哪种颜色的灯（蓝方=blue）；找不到再退回任意亮色")
    ap.add_argument("--v-thresh", type=int, default=150)
    ap.add_argument("--s-thresh", type=int, default=80)
    ap.add_argument("--min-area", type=float, default=20.0)
    ap.add_argument("--max-area", type=float, default=2000.0)
    ap.add_argument("--min-side", type=int, default=4)
    ap.add_argument("--max-aspect", type=float, default=5.0)
    ap.add_argument("--downscale", type=int, default=2)
    ap.add_argument("--distance", type=float, default=1.0)
    ap.add_argument("--topic", default="/image_raw", help="订阅的图像话题（bag 回放时用 /bag/image_raw）")
    ap.add_argument("--mode", choices=["incremental", "absolute"], default="incremental",
                    help="incremental=闭环增量（真实相机用）；absolute=按像素偏移直接换算角度（回放 bag 用）")
    ap.add_argument("--fov-deg", type=float, default=45.0, help="absolute 模式的水平视场角")
    ap.add_argument("--yaw-center", type=float, default=0.0, help="absolute 模式的图像中心对应 yaw")
    ap.add_argument("--search-rate", type=float, default=12.0,
                    help="找不到目标时 yaw 搜索速度（度/秒），0=不搜索")
    ap.add_argument("--pitch-base", type=float, default=90.0,
                    help="pitch 基准（度）；机体枪口有安装偏置时改这里，默认 90")
    ap.add_argument("--debug-image", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = FollowTarget(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):  # noqa: BLE001
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
        print(f"[follow_target] 结束：frames={node.frames} found={node.found}")


if __name__ == "__main__":
    main()
