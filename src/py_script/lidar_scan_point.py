import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_sub')
        self.sub = self.create_subscription(
            PointCloud2, '/lidar_scan/points', self.cb, 10
        )

    def cb(self, msg):
        self.get_logger().info(f"Got {msg.width} points")
        point_step = msg.point_step  # 18
        data = msg.data

        # 解析前5个点
        for i in range(min(540, msg.width)):
            start = i * point_step
            x = struct.unpack('<f', data[start:start+4])[0]
            y = struct.unpack('<f', data[start+4:start+8])[0]
            z = struct.unpack('<f', data[start+8:start+12])[0]
            print(f"Point {i}: ({x:.3f}, {y:.3f}, {z:.3f})")

        rclpy.shutdown()

def main():
    rclpy.init()
    node = PointCloudSubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()