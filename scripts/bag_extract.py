#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bag_extract.py —— 从 ros2 bag 里抽帧成图片，肉眼检查场景动作

用途：
  录完 bag 后不用开 rqt/foxglove，直接把 /image_raw 抽成 JPG，
  一眼确认"假人到底有没有按场景动"（尤其 D 遮挡、E 斜视）。

用法：
  python3 bag_extract.py /media/chen/RJ/rm_vision/bags/sceneB
  python3 bag_extract.py <bag> --every 30 --out ~/bag_frames/sceneB
  python3 bag_extract.py <bag> --max 40        # 最多存 40 张

输出：
  <out>/frame_<序号>_<时间戳ns>.jpg
  <out>/summary.txt   （话题、帧率估计、分辨率、编码）
"""
import argparse
import os
import sys

import cv2
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def open_reader(uri: str):
    storage_options = rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def to_bgr(msg: Image):
    h, w = msg.height, msg.width
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        n = h * w * 3
        if buf.size < n:
            return None
        arr = buf[:n].reshape(h, w, 3)
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if enc == "rgb8" else arr
    if enc in ("mono8", "8uc1"):
        n = h * w
        if buf.size < n:
            return None
        return cv2.cvtColor(buf[:n].reshape(h, w), cv2.COLOR_GRAY2BGR)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="bag 目录（含 metadata.yaml）")
    ap.add_argument("--topic", default="/image_raw")
    ap.add_argument("--out", default="")
    ap.add_argument("--every", type=int, default=30, help="每隔多少帧存一张（默认 30）")
    ap.add_argument("--max", type=int, default=60, help="最多存多少张（默认 60）")
    args = ap.parse_args()

    if not os.path.isfile(os.path.join(args.bag, "metadata.yaml")):
        sys.exit(f"[X] {args.bag} 不是 bag 目录（缺 metadata.yaml）")

    out_dir = args.out or os.path.join(
        os.path.expanduser("~/bag_frames"), os.path.basename(os.path.normpath(args.bag))
    )
    os.makedirs(out_dir, exist_ok=True)

    try:
        reader = open_reader(args.bag)
    except Exception as e:  # noqa: BLE001
        sys.exit(
            f"[X] 打开 bag 失败：{e}\n"
            "    如果是 --compression-mode file 的压缩 bag，先转一份未压缩的：\n"
            f"    ros2 bag convert -i {args.bag} -o /tmp/uncompressed\n"
        )

    idx = 0          # 图像消息序号
    saved = 0
    first_stamp = None
    last_stamp = None
    size = None
    encoding = None
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic != args.topic:
            continue
        idx += 1
        if first_stamp is None:
            first_stamp = stamp
        last_stamp = stamp
        if saved >= args.max:
            continue
        if idx != 1 and (idx - 1) % max(1, args.every) != 0:
            continue
        msg = deserialize_message(raw, Image)
        size = (msg.width, msg.height)
        encoding = msg.encoding
        img = to_bgr(msg)
        if img is None:
            print(f"[!] 第 {idx} 帧编码 {msg.encoding} 暂不支持，跳过")
            continue
        path = os.path.join(out_dir, f"frame_{idx:05d}_{stamp}.jpg")
        cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        saved += 1
        if saved % 10 == 0:
            print(f"    已保存 {saved} 张…")

    dur = (last_stamp - first_stamp) / 1e9 if (first_stamp and last_stamp) else 0
    fps = (idx - 1) / dur if dur > 0 else 0
    lines = [
        f"bag        : {args.bag}",
        f"topic      : {args.topic}",
        f"图像帧数   : {idx}",
        f"时长       : {dur:.2f} s",
        f"估算帧率   : {fps:.1f} Hz",
        f"分辨率     : {size[0]}x{size[1]}" if size else "分辨率     : 未知",
        f"编码       : {encoding}",
        f"已抽帧     : {saved} 张（every={args.every}）",
        f"输出目录   : {out_dir}",
    ]
    summary = "\n".join(lines)
    with open(os.path.join(out_dir, "summary.txt"), "w", encoding="utf-8") as f:
        f.write(summary + "\n")
    print()
    print(summary)


if __name__ == "__main__":
    main()
