"""S0-2: 订阅 /image_raw，打印分辨率与帧率（第一帧立即打印）。"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageProbe(Node):
    def __init__(self):
        super().__init__('image_probe')
        self.count = 0
        self.win_count = 0
        self.t0 = self.get_clock().now()
        self.t_last = self.get_clock().now()
        self.create_subscription(Image, '/image_raw', self.on_image, 10)
        self.get_logger().info('image_probe 已启动，等待 /image_raw ...')

    def on_image(self, msg):
        self.count += 1
        self.win_count += 1
        if self.count <= 3:
            self.get_logger().info(
                'FIRST#%d resolution=%dx%d encoding=%s frame_id=%s'
                % (self.count, msg.width, msg.height,
                   msg.encoding, msg.header.frame_id))
        now = self.get_clock().now()
        dt = (now - self.t_last).nanoseconds / 1e9
        if dt >= 2.0:
            total = (now - self.t0).nanoseconds / 1e9
            self.get_logger().info(
                'resolution=%dx%d | frames=%d avg=%.2fHz window=%.2fHz'
                % (msg.width, msg.height, self.count,
                   self.count / total if total > 0 else 0.0,
                   self.win_count / dt))
            self.win_count = 0
            self.t_last = now


def main():
    rclpy.init()
    node = ImageProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
