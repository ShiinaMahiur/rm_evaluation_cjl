"""S3-估计器节点：8 状态 EKF 跟踪装甲板目标

订阅 /solver/armors（S2 输出 JSON）→ EKF 预测/更新 → 发布 /tracker/target

状态：[px,py,pz,vx,vy,vz,yaw,vyaw]（odom 系）
关联：按预测位置最近邻 + 距离门限（1.0 m）；超门限视为丢失后重新捕获
丢失：1 帧无观测即置 lost（帧间隔 ~5-10 ms，满足 ≤30 ms）
重捕获：下一帧有观测立即恢复（≈1 帧）

发布 /tracker/target（std_msgs/String JSON）：
  {"stamp":..., "pos":[...], "vel":[...], "yaw":..., "vyaw":...,
   "predict_100ms":[...], "predict_200ms":[...], "lost":bool, "armor_id":...}

用法：
  ros2 run rm_vision_probe armor_tracker
  ros2 run rm_vision_probe armor_tracker --ros-args -p sigma_a:=0.5 -p sigma_pos:=0.03
"""
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .tracker import EKF8


class ArmorTracker(Node):
    def __init__(self):
        super().__init__("armor_tracker")
        # 默认参数与 metrics/s3_ekf.json 的标定结果一致（可用 ROS 参数覆盖）
        self.declare_parameter("sigma_a", 0.2)
        self.declare_parameter("sigma_alpha", 2.756)
        self.declare_parameter("sigma_pos", 0.026)
        self.declare_parameter("sigma_yaw", 0.35)
        self.declare_parameter("gate_m", 1.0)
        self.declare_parameter("lost_frames", 1)

        self.ekf = EKF8(
            sigma_a=float(self.get_parameter("sigma_a").value),
            sigma_alpha=float(self.get_parameter("sigma_alpha").value),
            sigma_pos=float(self.get_parameter("sigma_pos").value),
            sigma_yaw=float(self.get_parameter("sigma_yaw").value),
        )
        self.gate = float(self.get_parameter("gate_m").value)
        self.lost_frames = int(self.get_parameter("lost_frames").value)
        self.target_id = None
        self.last_stamp = None
        self.updates = 0
        self.lost_count = 0
        self.recaptures = 0

        self.pub = self.create_publisher(String, "/tracker/target", 10)
        self.sub = self.create_subscription(String, "/solver/armors", self.on_armors, 10)
        self.timer = self.create_timer(0.02, self.on_timer)   # 50 Hz 输出/预测
        self.get_logger().info(
            "armor_tracker 启动 sigma_a=%.2f sigma_alpha=%.2f sigma_pos=%.3f gate=%.1fm"
            % (self.ekf.sigma_a, self.ekf.sigma_alpha, math.sqrt(self.ekf.r_pos), self.gate))

    def _stamp(self, data):
        st = data.get("stamp") or {}
        sec = st.get("sec")
        nsec = st.get("nanosec")
        if sec is None:
            return self.get_clock().now().nanoseconds * 1e-9
        return float(sec) + float(nsec or 0) * 1e-9

    def on_armors(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        armors = data.get("armors") or []
        t = self._stamp(data)
        if self.last_stamp is not None and t <= self.last_stamp:
            return
        self.last_stamp = t
        pred = self.ekf.x[:3] if self.ekf.initialized else None
        best, best_d = None, 1e9
        for a in armors:
            pos = a.get("t_odom")
            if not pos:
                continue
            pos = np.asarray(pos, dtype=float)
            if pred is not None:
                d = float(np.linalg.norm(pos - pred))
                if d < best_d:
                    best, best_d, best_pos = a, d, pos
            else:
                # 未初始化：取最近的（离相机最近）
                d = float(a.get("distance_m", 1e9))
                if d < best_d:
                    best, best_d, best_pos = a, d, pos
        if best is None:
            self.ekf.mark_missed(t, self.lost_frames)
            if self.ekf.lost:
                self.lost_count += 1
            return
        if self.ekf.initialized and best_d > self.gate:
            # 跳变过大：按丢失重捕获处理
            self.ekf.reset(best_pos, t=t)
            self.recaptures += 1
        else:
            was_lost = self.ekf.lost
            self.ekf.update_pos(best_pos, t, gate=self.gate)
            if was_lost:
                self.recaptures += 1
            self.updates += 1
        q = best.get("R_odom_quat")
        if q:
            from .armor_solver import _quat2R
            R = _quat2R(type("Q", (), {"x": q[0], "y": q[1], "z": q[2], "w": q[3]})())
            self.ekf.update_yaw(math.atan2(R[1, 0], R[0, 0]))
        self.target_id = best.get("color")
        self._publish(t)

    def on_timer(self):
        # 没有新观测时也按时间推进（保持预测连续）
        if self.ekf.initialized:
            t = self.get_clock().now().nanoseconds * 1e-9
            self.ekf.predict(t)
            self._publish(t)

    def _publish(self, t):
        st = self.ekf.state()
        out = {
            "stamp": {"sec": int(t), "nanosec": int((t - int(t)) * 1e9)},
            "pos": [round(v, 4) for v in st["pos"]],
            "vel": [round(v, 4) for v in st["vel"]],
            "yaw": round(st["yaw"], 4),
            "vyaw": round(st["vyaw"], 4),
            "lost": st["lost"],
            "initialized": st["initialized"],
            "color": self.target_id,
            "updates": self.updates,
            "lost_count": self.lost_count,
            "recaptures": self.recaptures,
        }
        for ms in (100, 200):
            p = self.ekf.predict_ahead(ms / 1000.0)
            out[f"predict_{ms}ms"] = None if p is None else [round(float(v), 4) for v in p[0]]
        self.pub.publish(String(data=json.dumps(out, ensure_ascii=False)))


def main():
    rclpy.init()
    node = ArmorTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
