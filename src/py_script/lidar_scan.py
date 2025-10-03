import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar_scan',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f"Received {len(msg.ranges)} points")

        # 定义四个主要方向（基于机器人本体坐标系）
        # 假设机器人朝向 +X（在 Gazebo ENU 中 = 正东），则：
        directions = {
            "East (0°)": 0.0,               # 前方（+X）
            "North (+90°)": math.pi / 2,    # 左侧（+Y）
            "West (180°)": math.pi,         # 后方（-X）—— 注意：可能超出扫描范围
            "South (-90°)": -math.pi / 2,   # 右侧（-Y）
        }

        for name, target_angle in directions.items():
            # 计算对应索引（四舍五入更准确）
            index = int(round((target_angle - msg.angle_min) / msg.angle_increment))
            
            if 0 <= index < len(msg.ranges):
                distance = msg.ranges[index]
                actual_angle = msg.angle_min + index * msg.angle_increment
                print(f"{name}: {distance:.3f} m "
                      f"(index={index}, actual_angle={actual_angle:.3f} rad ≈ {math.degrees(actual_angle):.1f}°)")
            else:
                print(f"{name}: OUT OF SCAN RANGE (index={index} is invalid)")

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin_once(node, timeout_sec=2.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()