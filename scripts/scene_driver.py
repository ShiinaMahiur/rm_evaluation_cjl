#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene_driver.py —— S0 场景闭环驱动（读 /tf 测距测速 + xdotool 按键）

为什么需要它：
  用 `xdotool keydown` 按固定时长去造场景，实际速度/距离完全靠手感——
  2026-11-05 自查发现旧 bag 里 B 场景实测最大速度只有 0.01 m/s、A 场景距离
  只有 1.5~2.4 m，和考核定义（B 2 m/s 平移、A 3/6/10 m 三档）不符。
  本脚本改成**闭环**：从 /tf 里实时算距离/速度，按键按误差反馈，达标才继续。

两种模式：
  distance：把「相机 → 目标装甲板」的距离开到指定档位，然后静止停 dwell 秒
            （用于 A：3 / 6 / 10 m 三档静止）
  speed   ：让目标保持约 target m/s 的横向平移（沿相机视线垂直方向来回，
            用来把目标限制在画面内），用于 B（~2 m/s）

做法：
  1. 启动时先**标定按键方向**：逐个别按 I/J/K/L（假人）和 W/A/S/D（己方），
     分别量位移方向，得到「键 → 世界方向」映射（不同场景朝向不一样，不能写死）
  2. distance：选与「远离目标」方向最接近的己方键，闭环按到目标距离 ±tol
  3. speed  ：选与「切向」最接近的假人键，用滞环控制在目标速度附近，
             并在超过行程上限时反向（来回平移，不会跑出地图）
  4. 退出时释放所有按键（防止卡键导致仿真器一直动）

用法（一般由 record_bags.sh 调用）：
  python3 scene_driver.py --mode distance --targets 3 6 10 --dwell 6
  python3 scene_driver.py --mode speed --target 2.0 --duration 20
  python3 scene_driver.py --mode calibrate            # 只标定并打印映射
"""
import argparse
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage

SELF_KEYS = {"w": "w", "a": "a", "s": "s", "d": "d"}
REMOTE_KEYS = {"i": "i", "j": "j", "k": "k", "l": "l"}
CHAIN = [("map", "odom"), ("odom", "gimbal_link"),
         ("gimbal_link", "camera_link"), ("camera_link", "camera_optical_frame")]


def quat2R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class SceneDriver(Node):
    def __init__(self):
        super().__init__("scene_driver")
        self.tf = {}
        self.create_subscription(TFMessage, "/tf", self._on_tf, qos_profile_sensor_data)
        self.pressed = set()
        self.dummy_ids = None

    def _on_tf(self, msg):
        self.tf = {(t.header.frame_id, t.child_frame_id): t.transform for t in msg.transforms}

    # ---------- 基础量测 ----------
    def camera_pos(self):
        T = np.eye(4)
        for k in CHAIN:
            tr = self.tf.get(k)
            if tr is None:
                return None
            M = np.eye(4)
            M[:3, :3] = quat2R(tr.rotation)
            M[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
            T = T @ M
        return T[:3, 3]

    def armor_positions(self):
        out = {}
        for (p, c), tr in self.tf.items():
            if p != "map" or not c.startswith("armor_"):
                continue
            out[c] = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        return out

    def identify_target(self, probe_key="j"):
        """找出「假人」那台车的装甲板 id：按一下远端键，看哪些装甲板的世界位置变了

        起因：最近的装甲板往往是**旁边静止的队友/敌人**，不是被遥控的假人；
        直接取最近会锁错目标（实测标定位移恒为 0）。
        """
        before = self.armor_positions()
        if not before:
            return None
        self.key_down(probe_key)
        self.spin_for(0.6)
        self.key_up(probe_key)
        self.spin_for(0.4)
        after = self.armor_positions()
        moved = []
        for k, p0 in before.items():
            p1 = after.get(k)
            if p1 is None:
                continue
            d = float(np.linalg.norm(p1 - p0))
            if d > 0.05:
                moved.append((k, d))
        if not moved:
            return None
        moved.sort(key=lambda kv: -kv[1])
        ref = after[moved[0][0]]
        ids = [k for k, _ in moved if np.linalg.norm(after[k] - ref) < 0.9]
        if len(ids) < 2:
            ids = [moved[0][0]]
        print(f"[目标] 假人装甲板: {sorted(ids)}（探测位移 {moved[0][1]:.2f} m）", flush=True)
        return sorted(ids)

    def target_center(self, ids=None):
        """主目标：离相机最近、且不在自己身上的那一块装甲板（返回 位置, 名字）

        注意：不能用「附近所有装甲板取平均」——旁边还有别的机器人时会把运动稀释掉
        （实测 6 块板平均后位移只剩 0.02 m，导致标定失败）。
        """
        pos = self.armor_positions()
        cam = self.camera_pos()
        if cam is None:
            return None, None
        ids = ids or self.dummy_ids
        if ids:
            cand = [(i, pos[i]) for i in ids if i in pos]
        else:
            cand = [(i, p) for i, p in pos.items() if np.linalg.norm(p - cam) > 1.0]
        if not cand:
            return None, None
        name, p = min(cand, key=lambda kv: np.linalg.norm(kv[1] - cam))
        return p, name

    def distance(self):
        cam = self.camera_pos()
        c, _ = self.target_center()
        if cam is None or c is None:
            return None
        return float(np.linalg.norm(c - cam))

    # ---------- 按键 ----------
    def key_down(self, k):
        if k not in self.pressed:
            subprocess.run(["xdotool", "keydown", k], check=False)
            self.pressed.add(k)

    def key_up(self, k):
        if k in self.pressed:
            subprocess.run(["xdotool", "keyup", k], check=False)
            self.pressed.discard(k)

    def release_all(self):
        for k in list(self.pressed):
            self.key_up(k)

    # ---------- 等待 / 采样 ----------
    def spin_for(self, sec):
        t_end = time.time() + sec
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def sample_motion(self, key, which="remote", dur=0.55, settle=0.5):
        """按某个键 dur 秒，测量该对象的世界位移方向"""
        before = self.camera_pos() if which == "self" else self.target_center()[0]
        self.key_down(key)
        self.spin_for(dur)
        self.key_up(key)
        after = self.camera_pos() if which == "self" else self.target_center()[0]
        self.spin_for(settle)
        if before is None or after is None:
            return None
        d = after - before
        n = np.linalg.norm(d)
        return d / n if n > 1e-3 else None

    def calibrate(self, direction="speed"):
        """标定按键方向；direction=speed 用假人键，distance 用己方键"""
        keys = REMOTE_KEYS if direction == "speed" else SELF_KEYS
        which = "remote" if direction == "speed" else "self"
        mapping = {}
        print(f"[标定] 按键方向（{which}）...", flush=True)
        for k in keys:
            v = self.sample_motion(k, which=which)
            if v is not None:
                mapping[k] = v
                print(f"  {k}: 位移方向 {np.round(v, 2)}", flush=True)
        return mapping


def pick_key(mapping, want):
    """挑与目标方向 dot 最大的按键"""
    best, best_dot = None, -2
    for k, v in mapping.items():
        d = float(np.dot(v, want))
        if d > best_dot:
            best, best_dot = k, d
    return best, best_dot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["calibrate", "distance", "speed"], required=True)
    ap.add_argument("--targets", nargs="+", type=float, default=[3.0, 6.0, 10.0])
    ap.add_argument("--dwell", type=float, default=6.0, help="每档距离静止保持秒数")
    ap.add_argument("--tol", type=float, default=0.18, help="距离容差 (m)")
    ap.add_argument("--speed", type=float, default=2.0, help="B 场景目标速度 (m/s)")
    ap.add_argument("--duration", type=float, default=20.0, help="speed 模式持续秒数")
    ap.add_argument("--span", type=float, default=1.6, help="speed 模式来回半行程 (m)")
    ap.add_argument("--phase-timeout", type=float, default=15.0, help="distance 模式每档最长秒数")
    ap.add_argument("--rate", type=float, default=20.0, help="控制频率 Hz")
    args = ap.parse_args()

    rclpy.init()
    node = SceneDriver()
    # 等 /tf 有数据
    t0 = time.time()
    while time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.camera_pos() is not None:
            break
    if node.camera_pos() is None:
        print("[X] 等不到 /tf（仿真器没起？）", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(2)
    print(f"[ok] /tf 就绪，相机位置 {np.round(node.camera_pos(), 2)}", flush=True)

    period = 1.0 / max(args.rate, 1.0)
    try:
        node.dummy_ids = node.identify_target()
        if args.mode == "calibrate":
            node.calibrate("speed")
        elif args.mode == "speed":
            # ---- B：闭合控制目标速度，来回平移 ----
            mapping = {k: v for k, v in (node.calibrate("speed") or {}).items()
                       if v is not None}
            if len(mapping) < 2:
                print("[X] 标定失败（按键没生效？窗口没聚焦？）", file=sys.stderr)
                sys.exit(3)
            cam = node.camera_pos()
            tgt, _ = node.target_center()
            los = (tgt - cam)
            los[2] = 0.0
            los /= max(np.linalg.norm(los), 1e-6)
            tangent = np.array([-los[1], los[0], 0.0])   # 水平切向
            key_f, dot_f = pick_key(mapping, tangent)
            key_b, dot_b = pick_key(mapping, -tangent)
            print(f"[ok] 切向键: 正向 {key_f}(dot={dot_f:+.2f}) / 反向 {key_b}(dot={dot_b:+.2f})", flush=True)
            center0 = tgt.copy()
            cur_dir = 1
            key = key_f
            v_prev = None
            t_pos_prev = None
            t_end = time.time() + args.duration
            node.key_down(key)
            last_print = 0
            while time.time() < t_end:
                rclpy.spin_once(node, timeout_sec=period)
                c, _ = node.target_center()
                now = time.time()
                if c is None:
                    continue
                v = None
                if t_pos_prev is not None:
                    dt = now - t_pos_prev[0]
                    if dt > 1e-3:
                        v = float(np.linalg.norm(c - t_pos_prev[1]) / dt)
                t_pos_prev = (now, c.copy())
                if v is not None:
                    v = 0.7 * v + 0.3 * (v_prev if v_prev else v)
                    v_prev = v
                # 行程到头 / 撞墙卡住 → 反向（并缩短行程，场地小）
                off = float(np.dot(c - center0, tangent)) * cur_dir
                stuck = (v is not None and v < 0.35 and key in node.pressed)
                if stuck:
                    low_t = getattr(node, "_low_t", None) or now
                    node._low_t = low_t
                    stuck_long = (now - low_t) > 0.6
                else:
                    node._low_t = None
                    stuck_long = False
                if off > args.span or stuck_long:
                    node.key_up(key)
                    cur_dir *= -1
                    key = key_f if cur_dir > 0 else key_b
                    node.key_down(key)
                    if stuck_long:
                        args.span = max(0.6, args.span * 0.7)
                        print(f"[..] 撞墙反向（行程缩到 ±{args.span:.1f}m）→ 键 {key}", flush=True)
                        node._low_t = None
                    else:
                        print(f"[..] 反向（行程 {off:+.1f}m）→ 键 {key}", flush=True)
                # 速度滞环
                if v is None:
                    continue
                if v < args.speed - 0.35:
                    node.key_down(key)
                elif v > args.speed + 0.35:
                    node.key_up(key)
                if now - last_print > 2.0:
                    print(f"[B] t={args.duration - (t_end - now):4.1f}s 速度≈{v:.2f} m/s "
                          f"距离≈{np.linalg.norm(c - node.camera_pos()):.2f} m", flush=True)
                    last_print = now
            node.release_all()
        else:
            # ---- A：三档距离 + 静止保持 ----
            mapping = {k: v for k, v in (node.calibrate("distance") or {}).items()
                       if v is not None}
            if len(mapping) < 2:
                print("[X] 标定失败（按键没生效？窗口没聚焦？）", file=sys.stderr)
                sys.exit(3)
            for k, v in mapping.items():
                print(f"  {k}: {np.round(v, 2)}", flush=True)
            for target in args.targets:
                print(f"[A] 目标距离 {target:.1f} m ...", flush=True)
                t_start = time.time()
                key = None
                last_d, last_move_t = None, time.time()
                while time.time() - t_start < args.phase_timeout:
                    rclpy.spin_once(node, timeout_sec=0.02)
                    cam = node.camera_pos()
                    c, _ = node.target_center()
                    if cam is None or c is None:
                        continue
                    d = float(np.linalg.norm(c - cam))
                    err = d - target
                    if abs(err) <= args.tol:
                        if key is not None:
                            node.key_up(key)
                            key = None
                        break
                    # 卡住检测：2.5 s 内距离几乎不动就放弃这一档（撞墙/被挡）
                    if last_d is None or abs(d - last_d) > 0.05:
                        last_d, last_move_t = d, time.time()
                    elif time.time() - last_move_t > 2.5:
                        print(f"[A] 己方卡住（距离停在 {d:.2f} m，场地边界），尝试推开假人…", flush=True)
                        if key is not None:
                            node.key_up(key)
                            key = None
                        # 用假人键把目标继续推远（两头拉可以突破单侧场地限制）
                        if "remote_map" not in locals():
                            remote_map = {k: v for k, v in (node.calibrate("speed") or {}).items()
                                          if v is not None}
                        t_r = time.time()
                        last_d2, last_t2 = d, time.time()
                        while time.time() - t_r < args.phase_timeout:
                            rclpy.spin_once(node, timeout_sec=0.02)
                            cam = node.camera_pos()
                            c, _ = node.target_center()
                            if cam is None or c is None:
                                continue
                            d = float(np.linalg.norm(c - cam))
                            if abs(d - target) <= args.tol:
                                break
                            want = (c - cam) / max(d, 1e-6)
                            k2, _ = pick_key(remote_map, -want) if d < target else pick_key(remote_map, want)
                            if k2 != key:
                                if key is not None:
                                    node.key_up(key)
                                node.key_down(k2)
                                key = k2
                            if last_d2 is None or abs(d - last_d2) > 0.05:
                                last_d2, last_t2 = d, time.time()
                            elif time.time() - last_t2 > 2.5:
                                print(f"[A] 两头拉也卡住（{d:.2f} m），按实际距离录", flush=True)
                                break
                        if key is not None:
                            node.key_up(key)
                            key = None
                        break
                    # want = 相机 → 目标的方向；沿 -want 移动己方会让距离变大
                    want = (c - cam) / max(d, 1e-6)
                    if err < 0:
                        k2, _ = pick_key(mapping, -want)      # 太近 → 远离目标
                    else:
                        k2, _ = pick_key(mapping, want)       # 太远 → 靠近目标
                    if key != k2:
                        if key is not None:
                            node.key_up(key)
                        node.key_down(k2)
                        key = k2
                if key is not None:
                    node.key_up(key)
                node.spin_for(0.4)
                cam = node.camera_pos(); c, _ = node.target_center()
                d = float(np.linalg.norm(c - cam)) if cam is not None and c is not None else -1
                print(f"[A] 到达 {d:.2f} m（目标 {target:.1f}±{args.tol}），静止保持 {args.dwell:.0f}s", flush=True)
                node.spin_for(args.dwell)
                # 离目标太远说明场地不够大，后面的档位不用试了（避免录进废片段）
                if d < 0.85 * target:
                    print(f"[A] 实际只到 {d:.2f} m（场地限制），后续更大的距离跳过", flush=True)
                    break
    finally:
        node.release_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
