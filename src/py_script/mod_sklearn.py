'''
write: Liu Yongxue
email:805110687@qq.com
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude

import numpy as np
from sklearn.cluster import DBSCAN


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # === 订阅话题 ===
        self.scan_sub = self.create_subscription(LaserScan, '/lidar_scan', self.scan_cb, qos)
        self.localpos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                                                     self.vehicle_local_position_callback, qos)
        self.att_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                                self.vehicle_attitude_callback, qos)

        # === 最新状态缓存 ===
        self.latest_pos = None
        self.latest_att = None

        # === 障碍物跟踪器 ===
        # obstacle_id: {
        #   'pos': np.array([x, y, z]),
        #   'time': rclpy.time.Time,
        #   'vel': np.array([vx, vy, vz])
        # }
        self.obstacle_tracker = {}
        self.obstacle_id_counter = 0

        # === 参数 ===
        self.association_threshold = 1.0  # 障碍物关联距离阈值（米）
        self.alpha = 0.8  # 低通滤波系数（越大越平滑）

    def vehicle_local_position_callback(self, msg):
        self.latest_pos = msg

    def vehicle_attitude_callback(self, msg):
        self.latest_att = msg

    def scan_cb(self, msg: LaserScan):
        """LiDAR回调，负责障碍点识别、坐标变换与速度计算"""
        if self.latest_pos is None or self.latest_att is None:
            self.get_logger().warning("缺少位置或姿态数据，跳过当前帧。")
            return

        # 1️⃣ 从 LaserScan 提取点云 (FLU)
        points = self.preprocess_scan(msg)

        # 2️⃣ 聚类得到障碍物中心点
        centers_flu = self.cluster_points(points)

        # 3️⃣ 使用 LaserScan 时间戳计算速度
        # ROS2 时间戳: builtin_interfaces.msg.Time
        current_time = msg.header.stamp # 更精确的时间来源
        current_time_ros = self.get_clock().now()  # 用于日志打印（不参与计算）

        for center_flu in centers_flu:
            # 将二维点扩展为三维，z=0
            center_flu_3d = np.array([center_flu[0], center_flu[1], 0.0])

            # LiDAR 偏移和无人机中心高度（FLU 坐标系）
            lidar_offset = np.array([0.12, 0.0, 0.26])
            vehicle_center_height = np.array([0.0, 0.0, 0.24])
            center_flu_offset = center_flu_3d + lidar_offset + vehicle_center_height

            # 坐标变换 FLU → FRD → NED → ENU
            center_frd = self.flu_to_frd(center_flu_offset)
            center_ned_rel = self.body_to_ned(center_frd, self.latest_att)
            center_ned_abs = self.add_vehicle_pos(center_ned_rel, self.latest_pos)
            center_enu = self.ned_to_enu(center_ned_abs)

            # 跟踪与速度估计
            self.track_and_compute_speed(center_enu, current_time, current_time_ros)



    def preprocess_scan(self, msg: LaserScan):
        """将 LaserScan 转为 FLU 坐标点集"""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        return np.vstack((x, y)).T

    def cluster_points(self, points):
        """使用 DBSCAN 聚类识别障碍物中心"""
        if len(points) == 0:
            return []
        db = DBSCAN(eps=0.3, min_samples=3)
        labels = db.fit_predict(points)
        centers = []
        for label in set(labels):
            if label == -1:
                continue
            cluster = points[labels == label]
            centers.append(np.mean(cluster, axis=0))
        return centers

    def flu_to_frd(self, center):
        return np.array([center[0], -center[1], -center[2]])


    def body_to_ned(self, center_frd, att):
        """FRD → NED"""
        q = att.q  # [w, x, y, z]
        a, b, c, d = q[0], q[1], q[2], q[3]
        dcm = np.array([
            [1 - 2 * (c ** 2 + d ** 2), 2 * (b * c - a * d), 2 * (b * d + a * c)],
            [2 * (b * c + a * d), 1 - 2 * (b ** 2 + d ** 2), 2 * (c * d - a * b)],
            [2 * (b * d - a * c), 2 * (c * d + a * b), 1 - 2 * (b ** 2 + c ** 2)]
        ])
        return np.dot(dcm, center_frd)

    def add_vehicle_pos(self, ned_rel, pos):
        """相对NED + 飞行器位置 → 绝对NED"""
        vehicle_pos = np.array([pos.x, pos.y, pos.z])
        return vehicle_pos + ned_rel

    def ned_to_enu(self, ned):
        """NED → ENU"""
        return np.array([ned[1], ned[0], -ned[2]])

    def track_and_compute_speed(self, center_enu, current_time, current_time_ros):
        """障碍物跟踪 + 基于时间戳的速度估计 + 低通滤波"""
        # 查找最近的已有障碍物
        assigned_id, min_dist = None, float('inf')
        for oid, data in self.obstacle_tracker.items():
            dist = np.linalg.norm(center_enu - data['pos'])
            if dist < min_dist:
                min_dist = dist
                assigned_id = oid

        if min_dist < self.association_threshold and assigned_id is not None:
            # 已存在的障碍物 → 计算速度
            prev_data = self.obstacle_tracker[assigned_id]
            prev_pos = prev_data['pos']
            prev_time = prev_data['time']

            # 计算 dt (秒)
            dt = (current_time.sec - prev_time.sec) + (current_time.nanosec - prev_time.nanosec) / 1e9
            if dt <= 0:
                return

            # 新的速度估计
            v_new = (center_enu - prev_pos) / dt

            # === 低通滤波 ===
            if 'vel' in prev_data:
                v_filtered = self.alpha * prev_data['vel'] + (1 - self.alpha) * v_new
            else:
                v_filtered = v_new

            vx, vy, vz = v_filtered  # 解包三维速度
            speed = np.linalg.norm(v_filtered)

            # 输出信息
            self.get_logger().info(
            f"[Obstacle {assigned_id}] Pos ENU {center_enu}, "
            f"Speed={speed:.2f} m/s, "
            f"Vel (vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}), dt={dt:.2f}s"
        )

            # 更新跟踪数据
            self.obstacle_tracker[assigned_id]['pos'] = center_enu
            self.obstacle_tracker[assigned_id]['time'] = current_time
            self.obstacle_tracker[assigned_id]['vel'] = v_filtered

        else:
            # 新障碍物
            self.obstacle_tracker[self.obstacle_id_counter] = {
                'pos': center_enu,
                'time': current_time,
                'vel': np.zeros(3)
            }
            self.get_logger().info(f"🆕 新障碍物 {self.obstacle_id_counter}: Pos ENU {center_enu}")
            self.obstacle_id_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
