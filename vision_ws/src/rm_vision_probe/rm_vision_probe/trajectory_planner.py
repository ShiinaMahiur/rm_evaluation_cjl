"""S4-规划器节点：弹道解算 + 提前量 → /rm_gimbal/cmd + fire_advice

订阅：
  /tracker/target   (std_msgs/String JSON，S3 输出：pos/vel/lost)
  /muzzle_pose      (geometry_msgs/PoseStamped，发射点位置；缺失时用 /gimbal_pose)
发布：
  /rm_gimbal/cmd    (rm_interfaces/GimbalCmd：yaw/pitch 单位度，pitch=仰角，0=水平)
  /planner/aim      (std_msgs/String JSON：两种弹道方法对比、飞行时间、命中点)

参数（config/planner.yaml）：
  bullet_speed=25.0, gravity=9.81, method=exact|approx,
  fire_mode=single|auto|off, fire_interval=1.0, auto_rate=10.0,
  max_distance=12.0, min_distance=0.8, settle_time=0.35, angle_tol_deg=2.0
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from .planner import plan_aim, solve_ballistic

try:
    from rm_interfaces.msg import GimbalCmd
except Exception:                      # 没装 rm_interfaces 时只发 JSON
    GimbalCmd = None


class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__("trajectory_planner")
        p = self.declare_parameter
        p("bullet_speed", 25.0)
        p("gravity", 9.81)
        p("method", "exact")
        p("fire_mode", "single")       # single: 每 fire_interval 打一发；auto: 连发；off: 不开火
        p("fire_interval", 1.0)
        p("auto_rate", 10.0)
        p("max_distance", 12.0)
        p("min_distance", 0.8)
        p("settle_time", 0.35)
        p("lead_iters", 4)
        p("angle_tol_deg", 2.0)

        self.v = float(self.get_parameter("bullet_speed").value)
        self.g = float(self.get_parameter("gravity").value)
        self.method = str(self.get_parameter("method").value)
        self.fire_mode = str(self.get_parameter("fire_mode").value)
        self.fire_interval = float(self.get_parameter("fire_interval").value)
        self.auto_rate = float(self.get_parameter("auto_rate").value)
        self.max_d = float(self.get_parameter("max_distance").value)
        self.min_d = float(self.get_parameter("min_distance").value)
        self.settle = float(self.get_parameter("settle_time").value)
        self.lead_iters = int(self.get_parameter("lead_iters").value)
        self.angle_tol = float(self.get_parameter("angle_tol_deg").value)

        self.tracker = None
        self.tracker_t = 0.0
        self.muzzle = None
        self.last_fire = 0.0
        self.acquired_at = None
        self.shots = 0

        self.pub_cmd = self.create_publisher(GimbalCmd, "/rm_gimbal/cmd", 10) if GimbalCmd else None
        self.pub_aim = self.create_publisher(String, "/planner/aim", 10)
        self.create_subscription(String, "/tracker/target", self.on_tracker, 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)
        self.create_subscription(PoseStamped, "/muzzle_pose", self.on_muzzle, 10)
        self.create_timer(1.0 / 20.0, self.on_timer)
        self.get_logger().info(
            "trajectory_planner 启动 v=%.1f m/s g=%.2f method=%s fire=%s"
            % (self.v, self.g, self.method, self.fire_mode))

    # ---------- 输入 ----------
    def on_tracker(self, msg):
        try:
            self.tracker = json.loads(msg.data)
            self.tracker_t = self.get_clock().now().nanoseconds * 1e-9
            if self.acquired_at is None and not self.tracker.get("lost", True):
                self.acquired_at = self.tracker_t
        except json.JSONDecodeError:
            pass

    def on_muzzle(self, msg: PoseStamped):
        # 注意：仿真里 /muzzle_pose 是「自身坐标系下的恒等位姿」，不能用；以 /tf 为准
        pass

    def on_tf(self, msg: TFMessage):
        d = {(t.header.frame_id, t.child_frame_id): t.transform for t in msg.transforms}
        T = np.eye(4)
        for key in (("map", "odom"), ("odom", "gimbal_link"), ("gimbal_link", "muzzle")):
            tr = d.get(key)
            if tr is None:
                return
            M = np.eye(4)
            x, y, z, w = tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w
            M[:3, :3] = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ])
            M[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
            T = T @ M
        self.muzzle = T[:3, 3].copy()

    # ---------- 主循环 ----------
    def on_timer(self):
        if self.tracker is None or self.muzzle is None:
            return
        if self.tracker.get("lost", True) or not self.tracker.get("initialized", False):
            self.acquired_at = None
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.acquired_at is not None and now - self.acquired_at < self.settle:
            return
        pos = np.array(self.tracker.get("pos", []), dtype=float)
        vel = np.array(self.tracker.get("vel", []), dtype=float)
        if pos.size != 3 or vel.size != 3:
            return
        best = plan_aim(self.muzzle, pos, vel, self.v, self.g,
                        method=self.method, lead_iters=self.lead_iters)
        if best is None:
            return
        d = math.hypot(best["aim_point"][0] - self.muzzle[0],
                       best["aim_point"][1] - self.muzzle[1])
        if not (self.min_d <= d <= self.max_d):
            self._publish_aim(best, d, fire=False)
            return
        fire = self._fire_ready(now)
        if self.pub_cmd is not None:
            cmd = GimbalCmd()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.yaw = math.degrees(best["yaw"])
            cmd.pitch = math.degrees(best["pitch"])
            cmd.distance = float(d)
            cmd.fire_advice = bool(fire)
            self.pub_cmd.publish(cmd)
        if fire:
            self.shots += 1
        self._publish_aim(best, d, fire)

    def _fire_ready(self, now):
        if self.fire_mode == "off":
            return False
        if self.fire_mode == "single":
            if now - self.last_fire >= self.fire_interval:
                self.last_fire = now
                return True
            return False
        # auto：按 auto_rate 连续开火
        if now - self.last_fire >= 1.0 / max(self.auto_rate, 1e-3):
            self.last_fire = now
            return True
        return False

    def _publish_aim(self, best, d, fire):
        # 两种弹道方法对比（答辩/README 用）
        approx = solve_ballistic(self.muzzle, best["aim_point"], self.v, self.g, "approx")
        exact = solve_ballistic(self.muzzle, best["aim_point"], self.v, self.g, "exact")
        out = {
            "stamp": self.get_clock().now().nanoseconds,
            "yaw_deg": round(math.degrees(best["yaw"]), 2),
            "pitch_deg": round(math.degrees(best["pitch"]), 2),
            "t_flight_ms": round(best["t_flight"] * 1000.0, 1),
            "horizontal_distance_m": round(d, 2),
            "aim_point": [round(x, 3) for x in best["aim_point"]],
            "method": self.method,
            "pitch_deg_approx": None if approx is None else round(math.degrees(approx[1]), 3),
            "pitch_deg_exact": None if exact is None else round(math.degrees(exact[1]), 3),
            "fire_advice": bool(fire),
            "shots": self.shots,
        }
        self.pub_aim.publish(String(data=json.dumps(out, ensure_ascii=False)))


def main():
    rclpy.init()
    node = TrajectoryPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
