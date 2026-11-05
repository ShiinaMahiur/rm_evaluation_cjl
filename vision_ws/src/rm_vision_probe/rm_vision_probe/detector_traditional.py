"""S1-识别节点：传统灯条法检测装甲板（像素角点 + 颜色标签）

订阅 /image_raw                    → sensor_msgs/Image（rgb8）
发布 /detector/debug_image        → sensor_msgs/Image（bgr8，画好灯条/装甲板/统计）
发布 /detector/armors             → std_msgs/String（JSON：角点、颜色、置信分数、耗时）

用法：
  ros2 run rm_vision_probe detector_traditional
  ros2 run rm_vision_probe detector_traditional --ros-args -p debug_image:=false -p publish_hz:=10.0
"""
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .detector import DetectorParams, detect_image, draw_debug
from .solver import refine_corners


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_traditional")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("debug_image", True)
        self.declare_parameter("max_armors", 8)
        self.declare_parameter("v_thresh", 110)
        self.declare_parameter("s_thresh", 90)
        self.declare_parameter("subpixel", True)

        self.params = DetectorParams(
            max_armors=int(self.get_parameter("max_armors").value),
            v_thresh=int(self.get_parameter("v_thresh").value),
            s_thresh=int(self.get_parameter("s_thresh").value),
        )
        self.pub_armors = self.create_publisher(String, "/detector/armors", 10)
        self.pub_dbg = None
        if bool(self.get_parameter("debug_image").value):
            self.pub_dbg = self.create_publisher(Image, "/detector/debug_image", 10)
        self.sub = self.create_subscription(
            Image, "/image_raw", self.on_image, qos_profile_sensor_data)

        self.frames = 0
        self.armor_count = 0
        self.det_frames = 0
        self.last_pub = 0.0
        self.publish_period = 1.0 / max(float(self.get_parameter("publish_hz").value), 1e-3)
        self.stat_t0 = time.monotonic()
        self.stat_frames = 0
        self.stat_armors = 0
        self.stat_ms = 0.0
        self.get_logger().info(
            "detector_traditional 已启动（传统灯条法）publish_hz=%.1f" %
            float(self.get_parameter("publish_hz").value))

    def on_image(self, msg: Image):
        t0 = time.perf_counter()
        h, w = msg.height, msg.width
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if msg.encoding.lower() not in ("rgb8", "bgr8") or buf.size < h * w * 3:
            return
        img = buf[: h * w * 3].reshape(h, w, 3)
        if msg.encoding.lower() == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        else:
            img = img.copy()

        result = detect_image(img, self.params,
                              subpixel=bool(self.get_parameter("subpixel").value))
        cost_ms = (time.perf_counter() - t0) * 1000.0

        self.frames += 1
        self.stat_frames += 1
        self.stat_armors += len(result["armors"])
        self.stat_ms += cost_ms
        if result["armors"]:
            self.det_frames += 1
        self.armor_count += len(result["armors"])

        now = time.monotonic()
        if now - self.last_pub >= self.publish_period:
            self.last_pub = now
            payload = {
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                "frame_id": msg.header.frame_id,
                "armors": [
                    {"color": a["color"],
                     # 直接发「标定后的板外沿四角」（亚像素 + 灯条→板尺度展开），
                     # 下游 armor_solver 不需要再猜板尺寸，节点与离线评估口径一致
                     "corners": [[round(float(u), 2), round(float(v), 2)]
                                 for u, v in refine_corners(a, True)],
                     "calibrated": True,
                     "subpixel": a.get("subpix") is not None,
                     "center": [round(v, 2) for v in a["center"]],
                     "score": a["score"], "digit_score": a["digit_score"]}
                    for a in result["armors"]
                ],
                "cost_ms": round(cost_ms, 3),
            }
            self.pub_armors.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            if self.pub_dbg is not None:
                dbg = draw_debug(img, result, self.params)
                out = Image()
                out.header = msg.header
                out.height, out.width = dbg.shape[:2]
                out.encoding = "bgr8"
                out.is_bigendian = 0
                out.step = out.width * 3
                out.data = dbg.tobytes()
                self.pub_dbg.publish(out)

        if now - self.stat_t0 >= 2.0:
            dt = now - self.stat_t0
            self.get_logger().info(
                "frames=%d 处理帧率=%.1fHz 检出率=%.2f%% 平均装甲板=%.2f 平均耗时=%.2fms"
                % (self.frames, self.stat_frames / dt,
                   100.0 * self.det_frames / max(self.frames, 1),
                   self.stat_armors / max(self.stat_frames, 1),
                   self.stat_ms / max(self.stat_frames, 1)))
            self.stat_t0, self.stat_frames, self.stat_armors, self.stat_ms = now, 0, 0, 0.0


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("结束：frames=%d 检出帧=%d" % (node.frames, node.det_frames))
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
