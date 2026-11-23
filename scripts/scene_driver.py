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
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage

try:
    from rm_interfaces.msg import GimbalCmd
    HAVE_GIMBAL_MSG = True
except Exception:                     # 没装 rm_interfaces 时退化为「只测不瞄」
    GimbalCmd = None
    HAVE_GIMBAL_MSG = False

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


def quat2pitchyaw(q):
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) or 1.0
    x, y, z, w = q.x / n, q.y / n, q.z / n, q.w / n
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    return math.asin(sinp), yaw


class SceneDriver(Node):
    def __init__(self):
        super().__init__("scene_driver")
        self.tf = {}
        self.create_subscription(TFMessage, "/tf", self._on_tf, qos_profile_sensor_data)
        self.pressed = set()
        self.dummy_ids = None
        self.gimbal_pose = None
        self.K = None
        self.pub_cmd = self.create_publisher(GimbalCmd, "/rm_gimbal/cmd", 10) if HAVE_GIMBAL_MSG else None
        self.create_subscription(PoseStamped, "/gimbal_pose", self._on_gimbal_pose, 10)
        try:
            from sensor_msgs.msg import CameraInfo
            self.create_subscription(CameraInfo, "/camera_info", self._on_camera_info, 10)
        except Exception:
            pass

    def _on_gimbal_pose(self, msg):
        self.gimbal_pose = msg.pose

    def _on_camera_info(self, msg):
        self.K = list(msg.k)

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

    # ---------- 云台瞄准（用 /tf 算目标在相机系的角度，闭环发 /rm_gimbal/cmd）----------
    def _cam_vec(self):
        """目标在相机系的位置向量（相机系 x右 y下 z前）"""
        cam = self.camera_pos()
        tgt, _ = self.target_center()
        if cam is None or tgt is None:
            return None
        T = np.eye(4)
        for k in CHAIN:
            tr = self.tf.get(k)
            if tr is None:
                return None
            M = np.eye(4)
            M[:3, :3] = quat2R(tr.rotation)
            M[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
            T = T @ M
        return T[:3, :3].T @ (tgt - T[:3, 3]), tgt, cam

    def _desired_aim_deg(self):
        """由世界几何直接算「相机应该指向」的绝对 yaw / 仰角（不依赖当前朝向）"""
        cam = self.camera_pos()
        tgt, _ = self.target_center()
        if cam is None or tgt is None:
            return None
        d = tgt - cam
        horiz = math.hypot(d[0], d[1])
        yaw = math.degrees(math.atan2(d[1], d[0]))
        elev = math.degrees(math.atan2(d[2], max(horiz, 1e-6)))
        return yaw, elev, float(np.linalg.norm(d))

    def _wait_converged(self, timeout=0.6, eps=0.4):
        """等云台转到位：用相机系角误差「不再变化」判断（/gimbal_pose 是恒等四元数，不能用）"""
        last = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin_for(0.1)
            got = self._cam_vec()
            if got is None:
                continue
            v = got[0]
            if v[2] <= 0.1:
                continue
            e = (math.degrees(math.atan2(v[0], v[2])), math.degrees(math.atan2(v[1], v[2])))
            if last is not None and abs(e[0] - last[0]) < eps and abs(e[1] - last[1]) < eps:
                return True
            last = e
        return False

    def target_in_image(self, margin=40):
        """目标中心是否落在画面内（用 /camera_info 内参投影）→ (u, v, ok)"""
        got = self._cam_vec()
        if got is None or self.K is None:
            return None
        v = got[0]
        if v[2] <= 0.1:
            return (-1.0, -1.0, False)
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        u = fx * v[0] / v[2] + cx
        vv = fy * v[1] / v[2] + cy
        ok = (margin <= u <= 1440 - margin) and (margin <= vv <= 1080 - margin)
        return (float(u), float(vv), bool(ok))

    def aim_step_absolute(self):
        """发一次「指向目标」的绝对云台指令（不阻塞，20Hz 调也没问题）"""
        if self.pub_cmd is None:
            return False
        aim = self._desired_aim_deg()
        if aim is None:
            return False
        yaw_cmd, elev, dist = aim
        cmd = GimbalCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.yaw = float(yaw_cmd)
        cmd.pitch = float(elev)          # 云台 pitch = 仰角度数（0=水平），实测确认
        cmd.distance = float(dist)
        cmd.fire_advice = False
        self.pub_cmd.publish(cmd)
        return True

    def aim_until(self, tol=2.0, timeout=8.0, gain=0.7):
        """闭环把云台指向目标：世界几何给初值 → 发指令 → 等云台到位 → 相机系误差修正"""
        if self.pub_cmd is None:
            return False
        aim = self._desired_aim_deg()
        if aim is None:
            return False
        yaw_cmd, elev, dist = aim
        pitch_cmd = elev                 # 云台 pitch = 仰角度数（0=水平）
        sign_yaw = sign_pitch = 1.0
        t0 = time.time()
        prev = None
        flipped = False
        while time.time() - t0 < timeout:
            cmd = GimbalCmd()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.yaw, cmd.pitch, cmd.distance = float(yaw_cmd), float(pitch_cmd), float(dist)
            cmd.fire_advice = False
            self.pub_cmd.publish(cmd)
            self._wait_converged(timeout=0.8)
            got = self._cam_vec()
            if got is None:
                continue
            v = got[0]
            if v[2] <= 0.1:
                if not flipped:
                    yaw_cmd += 180.0
                    flipped = True
                    continue
                yaw_cmd += 25.0
                continue
            yaw_err = math.degrees(math.atan2(v[0], v[2]))
            pitch_err = math.degrees(math.atan2(v[1], v[2]))
            uv = self.target_in_image()
            if abs(yaw_err) <= tol and abs(pitch_err) <= tol and (uv is None or uv[2]):
                print(f"[瞄] 已居中（yaw {yaw_err:+.1f}° pitch {pitch_err:+.1f}°，"
                      f"像素 {None if uv is None else (round(uv[0]), round(uv[1]))}，距离 {dist:.2f} m）",
                      flush=True)
                return True
            if prev is not None:
                if abs(yaw_err) > abs(prev[0]) + 0.8:
                    sign_yaw = -sign_yaw
                if abs(pitch_err) > abs(prev[1]) + 0.8:
                    sign_pitch = -sign_pitch
            prev = (yaw_err, pitch_err)
            yaw_cmd += sign_yaw * gain * yaw_err
            pitch_cmd += sign_pitch * gain * pitch_err
        uv = self.target_in_image()
        print(f"[瞄] 超时（像素 {None if uv is None else (round(uv[0]), round(uv[1]), uv[2])}）", flush=True)
        return False

    def hold_aim(self, duration, hz=5.0, reaim_tol=5.0):
        """驻留期间保持瞄准：持续发绝对指令 + 检查目标是否还在画面里

        之前只在到位那一刻发一次，之后云台会漂、目标会出画（重录 A 后半段就是这么废的）。
        """
        t_end = time.time() + duration
        period = 1.0 / max(hz, 1.0)
        while time.time() < t_end:
            self.aim_step_absolute()
            self.spin_for(period)
            uv = self.target_in_image()
            if uv is None or uv[2]:
                continue
            # 目标出画了 → 重新瞄（最多耽误 1 秒）
            print(f"[瞄] 目标出画（像素 {round(uv[0])},{round(uv[1])}），重新对准", flush=True)
            self.aim_until(tol=reaim_tol, timeout=3.0)

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

    def probe_axes(self, keys, dur=1.2, settle=0.4):
        """逐键按住 dur 秒，返回 {key: (方向单位向量, 世界位移 m)} —— 用来找「哪个方向走得通」

        位移必须用**固定的一块装甲板**：用「最近装甲板」会因为运动中最近板切换而跳变，
        实测会得到 0.01 m 这种假行程（也正是之前选错轴的原因）。
        """
        out = {}
        fixed = (self.dummy_ids or [None])[0]
        for k in keys:
            def pos():
                if fixed:
                    p = self.armor_positions().get(fixed)
                    if p is not None:
                        return p
                return self.target_center()[0]
            p0 = pos()
            if p0 is None:
                continue
            self.key_down(k)
            self.spin_for(dur)
            self.key_up(k)
            self.spin_for(settle)
            p1 = pos()
            if p1 is None:
                continue
            d = p1 - p0
            n = float(np.linalg.norm(d))
            out[k] = (d / n if n > 1e-3 else np.zeros(3), n)
            print(f"  探测 {k}: 行程 {n:.2f} m", flush=True)
        return out

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
    ap.add_argument("--targets", nargs="+", type=float, default=[3.0, 6.0, 10.0])
    ap.add_argument("--dwell", type=float, default=6.0, help="每档距离静止保持秒数")
    ap.add_argument("--tol", type=float, default=0.18, help="距离容差 (m)")
    ap.add_argument("--speed", type=float, default=2.0, help="B 场景目标速度 (m/s)")
    ap.add_argument("--duration", type=float, default=20.0, help="speed 模式持续秒数")
    ap.add_argument("--span", type=float, default=1.2, help="speed 模式来回半行程 (m)")
    ap.add_argument("--phase-timeout", type=float, default=15.0, help="distance 模式每档最长秒数")
    ap.add_argument("--probe-dur", type=float, default=3.0, help="探测每个方向的按键时长 (s)")
    ap.add_argument("--axis-file", default="/tmp/scene_b_axis.json", help="prep 模式输出的轴信息")
    ap.add_argument("--mode", choices=["calibrate", "distance", "speed", "prep", "circle"], required=True)
    ap.add_argument("--radius", type=float, default=1.2, help="circle 模式圆周半径 (m)")
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
        node.aim_until(tol=3.0, timeout=4.0)
        if args.mode == "calibrate":
            node.calibrate("speed")
        elif args.mode == "prep":
            # ---- B 的准备工作：找主轴 + 回到起点，结果写文件 ----
            node.spin_for(2.0)          # 等仿真物理稳定（刚启动就发键会标定失败）
            mapping = {k: v for k, v in (node.calibrate("speed") or {}).items() if v is not None}
            if len(mapping) < 2:
                node.spin_for(1.5)
                mapping = {k: v for k, v in (node.calibrate("speed") or {}).items() if v is not None}
            if len(mapping) < 2:
                print("[X] 标定失败", file=sys.stderr); sys.exit(3)
            p_start = node.target_center()[0]
            print(f"[prep] 探测 {args.probe_dur:.1f}s/方向 ...", flush=True)
            probe = node.probe_axes(list(mapping.keys()), dur=args.probe_dur)
            # 选「单方向行程最长」的轴：反向键在走到远端后通常就通了
            # （从起点测反向会被身后的墙挡住，实测 k=0.12 而 i=2.55，双向取 min 会选错）
            key_f = max(probe, key=lambda k: probe[k][1])
            runway, axis = probe[key_f][1], probe[key_f][0]
            key_b = min(probe, key=lambda o: float(np.dot(probe[o][0], axis)))
            span = min(args.span, max(0.8, runway * 0.75))
            # 归位：朝探测前起点走回去（持续按住，边测边判）
            p_start_id = (node.dummy_ids or [None])[0]
            t_ret = time.time()
            while time.time() - t_ret < 10.0:
                rclpy.spin_once(node, timeout_sec=0.02)
                cur = node.armor_positions().get(p_start_id)
                if cur is None or p_start is None:
                    break
                back = p_start - cur
                if np.linalg.norm(back) < 0.25:
                    break
                kk, _ = pick_key({k: v[0] for k, v in probe.items()}, back)
                node.key_down(kk); node.spin_for(0.15); node.key_up(kk)
            node.release_all(); node.spin_for(0.4)
            cur = node.armor_positions().get(p_start_id)
            state = {"key_f": key_f, "key_b": key_b,
                     "axis": [float(x) for x in axis], "span": float(span),
                     "center": [float(x) for x in (cur if cur is not None else p_start)],
                     "fixed_id": p_start_id, "runway_m": float(runway),
                     "probe": {k: [float(x) for x in v[0]] for k, v in probe.items()}}
            import json as _json
            with open(args.axis_file, "w", encoding="utf-8") as f:
                _json.dump(state, f, ensure_ascii=False, indent=2)
            print(f"[prep] 轴 {key_f}/{key_b} 行程 {runway:.2f} m → span ±{span:.2f} m；已归位；"
                  f"写入 {args.axis_file}", flush=True)
        elif args.mode == "circle":
            # ---- B：匀速圆周平移（底盘自转 + 前进 → 半径 = v/ω，连续不掉头）----
            mapping = {k: v for k, v in (node.calibrate("speed") or {}).items() if v is not None}
            if len(mapping) < 2:
                print("[X] 标定失败", file=sys.stderr)
                sys.exit(3)
            fixed_id = (node.dummy_ids or [None])[0]
            def pos():
                return node.armor_positions().get(fixed_id) if fixed_id else None
            # 找「能一直走」的方向：试 1.2 s，选位移最大的键
            best_key, best_d = None, -1.0
            for k in mapping:
                p0 = pos()
                if p0 is None:
                    continue
                node.key_down(k); node.spin_for(1.2); node.key_up(k); node.spin_for(0.4)
                p1 = pos()
                if p1 is None:
                    continue
                d = float(np.linalg.norm(p1 - p0))
                print(f"  {k}: 1.2 s 走 {d:.2f} m", flush=True)
                if d > best_d:
                    best_key, best_d = k, d
            if best_key is None or best_d < 0.3:
                print("[X] 没有能走的方向", file=sys.stderr)
                sys.exit(3)
            print(f"[B] 圆周：前进键 {best_key}，半径 {args.radius:.1f} m，"
                  f"线速度 {args.speed:.1f} m/s（需 rotation_speed={args.speed/args.radius:.2f} rad/s）", flush=True)
            # 先按 u 打开假人底盘自转，再按住前进键 → 画圆
            subprocess.run(["xdotool", "key", "u"], check=False)
            node.spin_for(0.3)
            node.key_down(best_key)
            t_end = time.time() + args.duration
            last_print = 0
            while time.time() < t_end:
                node.aim_step_absolute()
                rclpy.spin_once(node, timeout_sec=period)
                now = time.time()
                if now - last_print > 2.0:
                    c = pos()
                    cam = node.camera_pos()
                    spd = None
                    print(f"[B] t={args.duration - (t_end - now):4.1f}s "
                          f"距离≈{np.linalg.norm(c - cam):.2f} m" if c is not None and cam is not None else "", flush=True)
                    last_print = now
            node.key_up(best_key)
            subprocess.run(["xdotool", "key", "u"], check=False)   # 关自转
            node.release_all()

        elif args.mode == "speed":
            # ---- B：沿 prep 选好的「双向轴」来回匀速平移 ----
            import json as _json
            use_file = args.axis_file and os.path.isfile(args.axis_file)
            if use_file:
                st = _json.load(open(args.axis_file, encoding="utf-8"))
                key_f, key_b = st["key_f"], st["key_b"]
                axis = np.array(st["axis"], dtype=float)
                span = float(st["span"])
                center0 = np.array(st["center"], dtype=float)
                fixed_id = st.get("fixed_id")
                runway = float(st.get("runway_m", span))
                print(f"[B] 用 prep 结果：轴 {key_f}/{key_b}，span ±{span:.2f} m（探测行程 {runway:.2f} m）",
                      flush=True)
            else:
                mapping = {k: v for k, v in (node.calibrate("speed") or {}).items() if v is not None}
                if len(mapping) < 2:
                    print("[X] 标定失败（按键没生效？窗口没聚焦？）", file=sys.stderr)
                    sys.exit(3)
                p_start = node.target_center()[0]
                probe = node.probe_axes(list(mapping.keys()), dur=args.probe_dur)
                if not probe:
                    print("[X] 探测失败", file=sys.stderr)
                    sys.exit(3)

                key_f = max(probe, key=lambda k: probe[k][1])
                runway, axis = probe[key_f][1], probe[key_f][0]
                key_b = min(probe, key=lambda o: float(np.dot(probe[o][0], axis)))
                span = min(args.span, max(0.8, runway * 0.75))
                fixed_id = (node.dummy_ids or [None])[0]
                t_ret = time.time()
                while time.time() - t_ret < 10.0:
                    rclpy.spin_once(node, timeout_sec=0.02)
                    cur = node.armor_positions().get(fixed_id)
                    if cur is None or p_start is None:
                        break
                    back = p_start - cur
                    if np.linalg.norm(back) < 0.25:
                        break
                    kk, _ = pick_key({k: v[0] for k, v in probe.items()}, back)
                    node.key_down(kk)
                    node.spin_for(0.15)
                    node.key_up(kk)
                node.release_all()
                node.spin_for(0.3)
                center0 = node.armor_positions().get(fixed_id)
                if center0 is None:
                    center0 = p_start
                print(f"[B] 双向轴 {key_f}/{key_b}，行程 ±{span:.1f} m（探测 {runway:.2f} m），已归位",
                      flush=True)
            fixed_id = fixed_id or (node.dummy_ids or [None])[0]
            # 起点可用性校验：prep 归位后可能正好贴住墙，先试探哪边真的能动
            def _pos_now():
                return node.armor_positions().get(fixed_id) if fixed_id else None
            usable = None
            for kk in (key_f, key_b):
                p0v = _pos_now()
                if p0v is None:
                    break
                node.key_down(kk); node.spin_for(0.8); node.key_up(kk); node.spin_for(0.35)
                p1v = _pos_now()
                if p1v is not None and float(np.linalg.norm(p1v - p0v)) > 0.15:
                    usable = kk
                    break
            if usable is None:
                print("[B] 警告：起点两个方向都走不动（贴墙），录出来的平移会很小", flush=True)
            else:
                key = usable
                cur_dir = 1 if usable == key_f else -1
                _c, _ = node.target_center()
                if _c is not None:
                    center0 = _c.copy()
                print(f"[B] 起点校验通过：先按 {usable} 方向开始", flush=True)
            if usable is None:
                key, cur_dir = key_f, 1
            else:
                key, cur_dir = usable, (1 if usable == key_f else -1)
            v_prev = None
            t_pos_prev = None
            t_end = time.time() + args.duration
            node.key_down(key)
            last_print = 0
            while time.time() < t_end:
                node.aim_step_absolute()
                rclpy.spin_once(node, timeout_sec=period)
                now = time.time()
                c = node.armor_positions().get(fixed_id) if fixed_id else None
                if c is None:
                    c, _ = node.target_center()
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
                off = float(np.dot(c - center0, axis)) * cur_dir
                stuck = (v is not None and v < 0.3 and key in node.pressed)
                if stuck:
                    low_t = getattr(node, "_low_t", None) or now
                    node._low_t = low_t
                    stuck_long = (now - low_t) > 0.7
                else:
                    node._low_t = None
                    stuck_long = False
                if off > span or stuck_long:
                    node.key_up(key)
                    cur_dir *= -1
                    key = key_f if cur_dir > 0 else key_b
                    node.key_down(key)
                    if stuck_long:
                        span = max(0.5, span * 0.7)
                        print(f"[B] 撞墙反向（行程缩到 ±{span:.1f} m）→ 键 {key}", flush=True)
                        node._low_t = None
                    else:
                        print(f"[B] 反向（行程 {off:+.1f} m）→ 键 {key}", flush=True)
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
                node.aim_until(tol=2.0, timeout=5.0)
                cam = node.camera_pos(); c, _ = node.target_center()
                d = float(np.linalg.norm(c - cam)) if cam is not None and c is not None else -1
                print(f"[A] 到达 {d:.2f} m（目标 {target:.1f}±{args.tol}），静止保持 {args.dwell:.0f}s", flush=True)
                node.hold_aim(args.dwell)
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
