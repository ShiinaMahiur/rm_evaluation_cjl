"""S0-3: 手动云台节点 —— 按指定 yaw/pitch（单位：度）发布 /rm_gimbal/cmd。

注意：Daedalus 用 GimbalAimTarget::from_solver_degrees() 解释该消息，
      所以 yaw / pitch 是「度」，不是弧度。
      distance = -1.0 表示"无解"，会让仿真清空跟踪目标。
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from rm_interfaces.msg import GimbalCmd


class GimbalCmdNode(Node):
    def __init__(self):
        super().__init__('gimbal_cmd')
        self.declare_parameter('yaw', 0.0)        # 度
        self.declare_parameter('pitch', 0.0)      # 度
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('distance', 3.0)   # 米，-1.0 = 无解
        self.declare_parameter('fire', False)

        rate = float(self.get_parameter('rate_hz').value)
        self.pub = self.create_publisher(GimbalCmd, '/rm_gimbal/cmd', 10)
        self.n = 0
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            'gimbal_cmd 已启动 rate=%.1fHz yaw=%.2f deg pitch=%.2f deg (单位: 度)'
            % (rate, self.get_parameter('yaw').value,
               self.get_parameter('pitch').value))

    def tick(self):
        msg = GimbalCmd()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gimbal_link'
        msg.yaw = float(self.get_parameter('yaw').value)
        msg.pitch = float(self.get_parameter('pitch').value)
        msg.yaw_diff = 0.0
        msg.pitch_diff = 0.0
        msg.distance = float(self.get_parameter('distance').value)
        msg.fire_advice = bool(self.get_parameter('fire').value)
        self.pub.publish(msg)
        self.n += 1
        if self.n % 100 == 0:
            self.get_logger().info(
                '已发布 %d 条 | yaw=%.2f pitch=%.2f distance=%.1f fire=%s'
                % (self.n, msg.yaw, msg.pitch, msg.distance, msg.fire_advice))


def main():
    rclpy.init()
    node = GimbalCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
