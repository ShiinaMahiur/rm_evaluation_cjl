#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""review_bags.py —— S0 数据集自查：A~E 每个 bag「到底拍到了什么」

看 bag 的两种方式：
  1) 实时回放（人眼）：`~/bag_view.sh <bag>`，或 `ros2 bag play` + `rqt_image_view`
  2) 批量自查（本脚本）：抽帧拼图 + 场景特征量化 + 与考核场景定义对拍

输出（默认 metrics/ 下）：
  metrics/s0_review.json              每场景量化指标 + 判定 + 每块装甲板统计
  metrics/s0_review/sceneX_sheet.jpg  8 帧拼图（画真值装甲板框和 id）

量化内容（全部来自 bag 的 /tf + /image_raw，不用检测器）：
  - 时长 / 图像帧数 / 帧率 / 话题
  - 每块装甲板：距离、世界系速度、yaw 角速度、视线夹角、可见率、遮挡事件
  - 主目标按场景规则挑选（A 面积最大、B 最快、C 转最快、D 遮挡最长、E 视角最斜）
  - 判定该场景是否符合考核定义（A 静止 / B 平移 / C 自转 / D 遮挡 / E 斜视）
  - 遮挡事件要求「前后都可见」的内部断档，避免把「一直不可见」误判成遮挡

用法：
  python3 scripts/review_bags.py                     # 审查 A~E
  python3 scripts/review_bags.py --scenes A B       # 只看指定场景
  python3 scripts/review_bags.py --sample-hz 10 --sheet-frames 8
"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "vision_ws", "src", "rm_vision_probe"))

from rm_vision_probe.solver import OBJ_PTS, plate_pose_from_marker  # noqa: E402

SCENE_DESC = {
    "A": "静止目标（3/6/10 m）",
    "B": "匀速平移 ~2 m/s",
    "C": "自转 + 平移",
    "D": "遮挡 0.5 s 后重现",
    "E": "大角度斜视 >60°",
}


def open_reader(bag_dir):
    style = "NONE"
    try:
        with open(os.path.join(bag_dir, "metadata.yaml"), encoding="utf-8") as fh:
            style = str(yaml.safe_load(fh)["rosbag2_bagfile_information"]
                        .get("compression_mode", "NONE")).upper()
    except Exception:
        pass
    r = (rosbag2_py.SequentialCompressionReader()
         if style in ("FILE", "MESSAGE") else rosbag2_py.SequentialReader())
    r.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    return r


def quat2R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tf_dict(msg):
    return {(t.header.frame_id, t.child_frame_id): t.transform for t in msg.transforms}


def compose(d, chain):
    T = np.eye(4)
    for k in chain:
        tr = d.get(k)
        if tr is None:
            return None
        M = np.eye(4)
        M[:3, :3] = quat2R(tr.rotation)
        M[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
        T = T @ M
    return T


def project(K, p_cam):
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    if p_cam[2] <= 0.05:
        return None
    return np.array([fx * p_cam[0] / p_cam[2] + cx, fy * p_cam[1] / p_cam[2] + cy])


def plate_infos(tf, T, K, w, h, min_dist=1.2, max_dist=14.0):
    """当前帧装甲板真值（板面中心/姿态/投影框/距离/视线夹角）"""
    Rc, tc = T[:3, :3], T[:3, 3]
    out = []
    for (parent, child), tr in tf.items():
        if parent != "map" or not child.startswith("armor_"):
            continue
        Rm = quat2R(tr.rotation)
        tm = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        dist = float(np.linalg.norm(tm - tc))
        if dist < min_dist or dist > max_dist:
            continue
        Rp, tp = plate_pose_from_marker(Rm, tm)
        pts, ok = [], True
        for op in OBJ_PTS:
            uv = project(K, Rc.T @ (tp + Rp @ op - tc))
            if uv is None:
                ok = False
                break
            pts.append(uv)
        if not ok:
            continue
        pts = np.array(pts)
        if pts[:, 0].min() < 0 or pts[:, 1].min() < 0 or pts[:, 0].max() > w or pts[:, 1].max() > h:
            continue
        v = tp - tc
        n = Rp[:, 2]
        cosang = abs(float(np.dot(n, v / max(np.linalg.norm(v), 1e-9))))
        ang = math.degrees(math.acos(max(-1.0, min(1.0, cosang))))
        out.append({"name": child, "R": Rp, "t": tp, "pts": pts, "dist": dist,
                    "angle": ang,
                    "area": float((pts[:, 0].max() - pts[:, 0].min())
                                  * (pts[:, 1].max() - pts[:, 1].min()))})
    return out


def lamp_visible(hsv, pts, min_px=4):
    x0, y0 = int(max(pts[:, 0].min(), 0)), int(max(pts[:, 1].min(), 0))
    x1, y1 = int(min(pts[:, 0].max(), hsv.shape[1])), int(min(pts[:, 1].max(), hsv.shape[0]))
    if x1 - x0 < 3 or y1 - y0 < 3:
        return False
    roi = hsv[y0:y1, x0:x1]
    s, v = roi[:, :, 1], roi[:, :, 2]
    return int(((v > 110) & (s > 90)).sum()) >= min_px


def rot_angle(A, B):
    d = (np.trace(A.T @ B) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, d))))


def stat(v):
    if not v:
        return None
    return {"mean": round(float(np.mean(v)), 3), "median": round(float(np.median(v)), 3),
            "p95": round(float(np.percentile(v, 95)), 3),
            "min": round(float(np.min(v)), 3), "max": round(float(np.max(v)), 3)}


def occlusion_events(timeline, t0):
    """timeline: [(t, visible)]，只统计「前后都可见」的内部断档（≥0.3 s）"""
    ev = []
    first_vis = next((i for i, (_, v) in enumerate(timeline) if v), None)
    last_vis = next((len(timeline) - 1 - i for i, (_, v) in enumerate(reversed(timeline)) if v), None)
    if first_vis is None:
        return ev
    run_start, run_dur = None, 0.0
    for i, (t, vis) in enumerate(timeline):
        if not vis and first_vis < i < last_vis:
            if run_start is None:
                run_start = t
            run_dur = t - run_start
        else:
            if run_start is not None and run_dur >= 0.3:
                ev.append({"start_s": round(run_start - t0, 2), "dur_s": round(run_dur, 2)})
            run_start, run_dur = None, 0.0
    return ev


def review_scene(bag_dir, scene, sample_hz, out_dir, sheet_n=8):
    reader = open_reader(bag_dir)
    meta = yaml.safe_load(open(os.path.join(bag_dir, "metadata.yaml"),
                               encoding="utf-8"))["rosbag2_bagfile_information"]
    duration = meta["duration"]["nanoseconds"] / 1e9
    topics = {t["topic_metadata"]["name"]: t["message_count"]
              for t in meta["topics_with_message_count"]}
    n_img = topics.get("/image_raw", 0)
    fps = n_img / duration if duration > 0 else 0.0
    step = max(1, int(round(fps / max(sample_hz, 0.5)))) if fps else 1
    sheet_positions = [int(round((i + 0.5) * n_img / sheet_n)) for i in range(sheet_n)]
    sheet_set = set(sheet_positions)

    tf, K = {}, None
    idx = 0
    frames = []           # [(t, {armor: st}, set(visible_ids))]
    sheet_frames = []     # [(t, bgr, [visible infos])]
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic == "/tf":
            tf = tf_dict(deserialize_message(raw, TFMessage))
            continue
        if topic == "/camera_info":
            K = list(deserialize_message(raw, CameraInfo).k)
            continue
        if topic != "/image_raw":
            continue
        idx += 1
        need_sheet = idx in sheet_set
        if not need_sheet and (idx - 1) % step:
            continue
        msg = deserialize_message(raw, Image)
        t = stamp * 1e-9
        h, w = msg.height, msg.width
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if buf.size < h * w * 3:
            continue
        rgb = buf[: h * w * 3].reshape(h, w, 3)
        T = compose(tf, [("map", "odom"), ("odom", "gimbal_link"),
                         ("gimbal_link", "camera_link"), ("camera_link", "camera_optical_frame")])
        if K is None or T is None:
            continue
        infos = plate_infos(tf, T, K, w, h)
        per, vis_ids = {}, set()
        if infos:
            hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
            for it in infos:
                it["visible"] = lamp_visible(hsv, it["pts"])
                per[it["name"]] = it
                if it["visible"]:
                    vis_ids.add(it["name"])
        if need_sheet:
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            sheet_frames.append((t, bgr, [per[k] for k in vis_ids]))
        frames.append((t, per, vis_ids))

    t0 = frames[0][0] if frames else 0.0
    all_ids = sorted({k for _, per, _ in frames for k in per})

    # ---- 每块装甲板统计（运动用 /tf 直接算，不受可见性影响）----
    per_armor = {}
    for aid in all_ids:
        timeline, states = [], []
        for t, per, vis_ids in frames:
            timeline.append((t, aid in vis_ids))
            if aid in per:
                states.append((t, per[aid]))
        speeds, yaws = [], []
        for (ta, a), (tb, b) in zip(states, states[1:]):
            dt = tb - ta
            if 0.02 <= dt <= 0.6:
                speeds.append(float(np.linalg.norm(b["t"] - a["t"]) / dt))
                yaws.append(rot_angle(b["R"], a["R"]) / dt)
        dists = [s["dist"] for _, s in states]
        angs = [s["angle"] for _, s in states]
        areas = [s["area"] for _, s in states]
        per_armor[aid] = {
            "distance_m": stat(dists), "speed_mps": stat(speeds),
            "yaw_rate_dps": stat(yaws), "view_angle_deg": stat(angs),
            "area_px": stat(areas),
            "visible_ratio": round(sum(1 for _, v in timeline if v) / max(len(timeline), 1), 4),
            "occlusion_events": occlusion_events(timeline, t0),
        }

    def pick(rule, min_vis=0.15):
        best, best_val = None, -1e9
        for aid, st in per_armor.items():
            if st["visible_ratio"] < min_vis:
                continue
            v = rule(st)
            if v is not None and v > best_val:
                best, best_val = aid, v
        return best

    if scene == "A":
        tgt = pick(lambda st: st["area_px"]["median"] if st["area_px"] else -1)
    elif scene == "B":
        tgt = pick(lambda st: st["speed_mps"]["median"] if st["speed_mps"] else -1)
    elif scene == "C":
        tgt = pick(lambda st: st["yaw_rate_dps"]["median"] if st["yaw_rate_dps"] else -1)
    elif scene == "D":
        tgt = pick(lambda st: max([e["dur_s"] for e in st["occlusion_events"]], default=0))
    else:
        tgt = pick(lambda st: st["view_angle_deg"]["median"] if st["view_angle_deg"] else -1)
    if tgt is None:
        tgt = max(all_ids, key=lambda a: per_armor[a]["visible_ratio"]) if all_ids else None
    target = per_armor.get(tgt, {})

    # ---- 场景判定 ----
    def g(fn):
        vals = [fn(st) for st in per_armor.values()]
        vals = [v for v in vals if v is not None]
        return max(vals) if vals else 0.0

    max_speed = g(lambda st: st["speed_mps"]["median"] if st["speed_mps"] else None)
    max_yaw = g(lambda st: st["yaw_rate_dps"]["median"] if st["yaw_rate_dps"] else None)
    max_occl = g(lambda st: max([e["dur_s"] for e in st["occlusion_events"]], default=None))
    max_angle = g(lambda st: st["view_angle_deg"]["median"] if st["view_angle_deg"] else None)
    dists_all = [st["distance_m"]["median"] for st in per_armor.values() if st["distance_m"]]
    dist_lo, dist_hi = (min(dists_all), max(dists_all)) if dists_all else (0, 0)

    if scene == "A":
        ok = max_speed < 0.3
        verdict = "符合（静止）" if ok else f"可疑（有运动 {max_speed:.2f} m/s）"
        detail = f"画面内装甲板距离 {dist_lo:.1f}~{dist_hi:.1f} m（考核要求 3/6/10 m 三档）"
    elif scene == "B":
        ok = 1.0 <= max_speed <= 3.5
        verdict = (f"符合（平移 {max_speed:.2f} m/s）" if ok
                   else f"可疑（最大速度仅 {max_speed:.2f} m/s）")
        detail = f"实测最大速度 {max_speed:.2f} m/s（要求 ~2 m/s）"
    elif scene == "C":
        ok = max_yaw > 15
        verdict = (f"符合（自转 {max_yaw:.0f} °/s）" if ok
                   else f"可疑（自转 {max_yaw:.0f} °/s 偏小）")
        detail = f"自转 {max_yaw:.0f} °/s + 平移 {max_speed:.2f} m/s"
    elif scene == "D":
        ok = max_occl >= 0.3
        verdict = f"符合（遮挡 {max_occl:.2f}s）" if ok else "可疑（没抓到内部遮挡）"
        detail = f"最长内部遮挡断档 {max_occl:.2f} s（要求 ~0.5 s）"
    elif scene == "E":
        ok = max_angle > 55
        verdict = (f"符合（视线夹角中位 {max_angle:.1f}°）" if ok
                   else f"可疑（夹角 {max_angle:.1f}° 不够）")
        detail = f"最大视线夹角中位 {max_angle:.1f}°（要求 >60°）"
    else:
        verdict, detail = "未定义", ""

    # ---- 拼图 ----
    sheet_path = None
    if sheet_frames:
        tiles = []
        for t, bgr, vis in sheet_frames[:sheet_n]:
            im = cv2.resize(bgr, (480, 360))
            sx, sy = 480 / bgr.shape[1], 360 / bgr.shape[0]
            for it in vis:
                pts = (it["pts"] * [sx, sy]).astype(np.int32)
                cv2.polylines(im, [pts], True, (0, 255, 0), 2)
                cv2.putText(im, it["name"].replace("armor_", "#"),
                            (int(pts[:, 0].min()), int(pts[:, 1].min()) - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(im, f"t={t - t0:.2f}s", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            tiles.append(im)
        while len(tiles) < sheet_n:
            tiles.append(np.zeros((360, 480, 3), np.uint8))
        rows = [np.hstack(tiles[i:i + 4]) for i in range(0, sheet_n, 4)]
        sheet = np.vstack(rows)
        cv2.putText(sheet, f"scene{scene} | {SCENE_DESC.get(scene,'')} | {verdict}",
                    (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        sheet_path = os.path.join(out_dir, f"scene{scene}_sheet.jpg")
        cv2.imwrite(sheet_path, sheet, [cv2.IMWRITE_JPEG_QUALITY, 85])

    return {
        "scene": scene, "desc": SCENE_DESC.get(scene, ""),
        "duration_s": round(duration, 2), "images": n_img, "fps": round(fps, 1),
        "topics": topics, "verdict": verdict, "detail": detail,
        "target_armor": tgt, "target": target,
        "scene_metrics": {
            "max_speed_mps": round(max_speed, 3),
            "max_yaw_rate_dps": round(max_yaw, 2),
            "max_occlusion_s": round(max_occl, 2),
            "max_view_angle_deg": round(max_angle, 2),
            "distance_range_m": [round(dist_lo, 2), round(dist_hi, 2)],
        },
        "per_armor": per_armor,
        "sheet": sheet_path,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags-dir", default="/media/chen/RJ/rm_vision/bags")
    ap.add_argument("--scenes", nargs="+", default=["A", "B", "C", "D", "E"])
    ap.add_argument("--sample-hz", type=float, default=10.0)
    ap.add_argument("--sheet-frames", type=int, default=8)
    ap.add_argument("--out-dir", default="metrics")
    ap.add_argument("--json-out", default="metrics/s0_review.json")
    args = ap.parse_args()

    img_out = os.path.join(args.out_dir, "s0_review")
    os.makedirs(img_out, exist_ok=True)
    report = {}
    for scene in args.scenes:
        bag = os.path.join(args.bags_dir, f"scene{scene}")
        if not os.path.isfile(os.path.join(bag, "metadata.yaml")):
            print(f"[跳过] 没有 {bag}")
            continue
        print(f"==> scene{scene} ...", flush=True)
        r = review_scene(bag, scene, args.sample_hz, img_out, args.sheet_frames)
        report[scene] = r
        print(f"    {r['verdict']}", flush=True)
        print(f"    {r['detail']}", flush=True)
        print(f"    主目标={r['target_armor']} 可见率={r['target'].get('visible_ratio')}", flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
    with open(args.json_out, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print("\nJSON:", args.json_out)
    print("拼图:", img_out)
    print()
    print(f"{'场景':<4}{'时长':<8}{'帧率':<8}{'判定':<32}{'关键指标'}")
    for s, r in report.items():
        m = r["scene_metrics"]
        print(f"{s:<5}{r['duration_s']:<8}{r['fps']:<8}{r['verdict']:<32}"
              f"速度 {m['max_speed_mps']} m/s | yaw {m['max_yaw_rate_dps']} °/s | "
              f"遮挡 {m['max_occlusion_s']} s | 夹角 {m['max_view_angle_deg']}° | "
              f"距离 {m['distance_range_m']}")


if __name__ == "__main__":
    main()
