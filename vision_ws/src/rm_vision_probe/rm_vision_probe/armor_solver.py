"""S2-观测器节点：装甲板角点 → PnP → 相机系/世界系位姿

订阅：
  /detector/armors   (std_msgs/String JSON，来自 detector_traditional)
  /camera_info       (sensor_msgs/CameraInfo)
  /tf                (tf2，用于把相机系位姿转到 odom 系)

发布：
  /solver/armors     (std_msgs/String JSON)：每块装甲板的
                     - 距离（m）、重投影误差（px）
                     - 相机系 t/R、odom 系 t/R
                     - 颜色、角点

用法：
  ros2 run rm_vision_probe armor_solver
  ros2 run rm_vision_probe armor_solver --ros-args -p aspect_fix:=false
"""
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
import tf2_ros

from .solver import refine_corners, rot_to_quat_msg, solve_pnp


class ArmorSolver(Node):
    def __init__(self):
        super().__init__("armor_solver")
        self.declare_parameter("aspect_fix", True)
        self.declare_parameter("max_reproj_px", 12.0)
        self.declare_parameter("min_distance", 0.5)
        self.declare_parameter("max_distance", 30.0)

        self.K = None
        self.D = None
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(CameraInfo, "/camera_info", self.on_cam_info, 10)
        self.create_subscription(String, "/detector/armors", self.on_armors, 10)
        self.pub = self.create_publisher(String, "/solver/armors", 10)

        self.frames = 0
        self.solved = 0
        self.rej = 0
        self.stat_t0 = time.monotonic()
        self.stat_n = 0
        self.get_logger().info(
            "armor_solver 已启动（PnP=SOLVEPNP_IPPE, aspect_fix=%s）" %
            bool(self.get_parameter("aspect_fix").value))

    def on_cam_info(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(list(msg.k), dtype=np.float64).reshape(3, 3)
            self.D = np.array(list(msg.d), dtype=np.float64) if msg.d else None
            self.get_logger().info(
                "camera_info: fx=%.1f fy=%.1f cx=%.1f cy=%.1f" %
                (self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]))

    def on_armors(self, msg: String):
        if self.K is None:
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.frames += 1
        t0 = time.perf_counter()
        out = []
        for a in data.get("armors", []):
            corners = refine_corners(a, self.get_parameter("aspect_fix").value)
            r = solve_pnp(corners, self.K, self.D)
            if r is None:
                self.rej += 1
                continue
            R, t, rep = r
            dist = float(np.linalg.norm(t))
            assert dist > 0
            if rep > float(self.get_parameter("max_reproj_px").value):
                self.rej += 1
                continue
            if not (float(self.get_parameter("min_distance").value) < dist <
                    float(self.get_parameter("max_distance").value)):
                self.rej += 1
                continue
            item = {
                "color": a.get("color"),
                "corners": [[round(float(x), 1), round(float(y), 1)] for x, y in corners],
                "distance_m": round(dist, 3),
                "reproj_px": round(rep, 3),
                "t_cam": [round(float(v), 4) for v in t],
                "R_cam_quat": rot_to_quat_msg(R),
            }
            try:
                tf = self.buf.lookup_transform("odom", "camera_optical_frame", rclpy.time.Time())
                Rc = _quat2R(tf.transform.rotation)
                tc = np.array([tf.transform.translation.x, tf.transform.translation.y,
                               tf.transform.translation.z])
                t_odom = Rc @ t + tc
                R_odom = Rc @ R
                item["t_odom"] = [round(float(v), 4) for v in t_odom]
                item["R_odom_quat"] = rot_to_quat_msg(R_odom)
            except Exception:
                pass
            out.append(item)
            self.solved += 1
        cost_ms = (time.perf_counter() - t0) * 1000.0
        payload = {
            "stamp": data.get("stamp"),
            "frame_id": data.get("frame_id"),
            "solver": "pnp_ippe",
            "yaw_deg": None,
            "armors": out,
            "solver_ms": round(cost_ms, 3),
        }
        self.pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

        self.stat_n += 1
        now = time.monotonic()
        if now - self.stat_t0 >= 2.0:
            dt = now - self.stat_t0
            self.get_logger().info(
                "frames=%d solved=%d rejected=%d 解算帧率=%.1fHz 平均解算=%.2fms"
                % (self.frames, self.solved, self.rej, self.stat_n / dt,
                   cost_ms))
            self.stat_t0, self.stat_n = now, 0


def _quat2R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = ArmorSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            "结束：frames=%d solved=%d rejected=%d" % (node.frames, node.solved, node.rej))
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
