'''
writer:Liu Yongxue
email: 805110687@qq.com
'''
#!/usr/bin/env python3
"""
ros2_px4_lidar_processor.py

功能说明：
- 将 2D LiDAR 扫描数据转换为世界坐标系（ENU）下的障碍物位置。
- 对障碍物进行聚类、跟踪、速度估计（含低通滤波）。
- 通过定时器（每1秒）统一计算速度，避免高频噪声。
- 使用 PX4 提供的位姿（位置 + 姿态）将 LiDAR 点从机体坐标系转换到全局 ENU。

设计分离：
- scan_cb：只做感知（聚类 + 坐标转换），不计算速度。
- track_and_compute_speed：在定时器中统一处理跟踪与速度估计，逻辑清晰、时序可控。
"""

'''Notes:
speed_window=3.0 表示用过去3秒的轨迹计算速度，适合中低速场景；高速场景可减小至 1.0。

场景	                         特点	         推荐 speed_window	    推荐定时器间隔 T	                   说明
低速/静态障碍物（如室内巡检）	障碍物移动慢，噪声主导	   2.0 ~ 5.0 s	        0.5 ~ 1.0 s	             大窗口平滑噪声，低频更新足够
中速（如室外无人机避障）	障碍物可能移动（行人、车辆）	1.0 ~ 2.0 s	        0.3 ~ 0.5 s	              平衡响应速度与平滑性
高速（如高速飞行、赛车）	动态剧烈，需快速响应	      0.5 ~ 1.0 s	       0.1 ~ 0.2 s（10~5Hz）	小窗口避免滞后，高频更新
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude


class LidarPX4Processor(Node):
    def __init__(self):
        super().__init__('lidar_px4_processor')

        # 设置 QoS 策略：与 PX4 的 FAST RTPS 通信兼容（BEST_EFFORT + VOLATILE）
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 不重传，适合高频传感器
            durability=DurabilityPolicy.VOLATILE,       # 不保留历史
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ========== 参数配置 ==========
        self.range_min = 0.1          # LiDAR 有效最小距离（米）
        self.range_max = 30.0         # LiDAR 有效最大距离（米）
        self.cluster_distance_threshold = 0.8  # 聚类时点间最大距离（米）
        self.min_cluster_size = 3     # 聚类最小点数（防噪点）
        self.obs_match_radius = 1.0   # 障碍物匹配最大距离（米）

        # ========== 订阅器 ==========
        self.scan_sub = self.create_subscription(LaserScan, '/lidar_scan', self.scan_cb, qos)
        self.localpos_sub = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, 
            qos
        )
        self.att_sub = self.create_subscription(
            VehicleAttitude, 
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback, 
            qos
        )

        # ========== 发布器 ==========
        # 发布格式：[x1, y1, z1, vx1, vy1, vz1, x2, y2, z2, vx2, ...]
        self.obs_pub = self.create_publisher(Float32MultiArray, '/obstacles_enu', 10)

        # ========== 状态变量 ==========
        self.latest_localpos = None   # 最新无人机 NED 位置
        self.latest_att_q = None      # 最新无人机四元数姿态 [x, y, z, w]

        self.prev_obstacles = {}      # 上一帧（或历史）障碍物：{id: {'pos', 't'}}
        self.curr_obstacles = []      # 当前帧障碍物列表（由 scan_cb 填充）
        self.next_obs_id = 0          # 下一个障碍物 ID

        # 定时器：每 0.5 秒调用一次 track_and_compute_speed
        # → 保证速度计算频率固定，避免回调抖动导致 dt 过小
        self.timer = self.create_timer(0.5, self.track_and_compute_speed)

        self.get_logger().info('lidar_px4_processor initialized')

    # ---------------- 回调函数 ----------------

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        """接收 PX4 本地位置（NED 坐标系）"""
        self.latest_localpos = msg

    def vehicle_attitude_callback(self, msg: VehicleAttitude):
        """接收 PX4 姿态四元数，转为 NumPy 数组 [w, x, y, z]"""
        try:
            self.latest_att_q = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])
        except Exception:
            self.get_logger().warning('VehicleAttitude message missing q field')

    def scan_cb(self, scan: LaserScan):
        """
        处理 LiDAR 扫描数据：
        1. 滤除无效点（inf, nan, 超出范围）
        2. 转换为 FLU 坐标系下的 2D 点云（z=0）
        3. 聚类（欧氏聚类）
        4. 对每个聚类中心：
            a. FLU → FRD（LiDAR 通常朝前，x前 y左 z上 → FRD: x前 y右 z下）
            b. 用当前姿态将 FRD 点旋转到 NED
            c. 加上无人机 NED 位置 → 障碍物 NED 位置
            d. NED → ENU（ENU: x东 y北 z上）
        5. 存入 self.curr_obstacles，供定时器使用
        """
        ranges = np.array(scan.ranges)
        angles = np.arange(len(ranges)) * scan.angle_increment + scan.angle_min
        # 有效点掩码：非 inf/nan 且在 [range_min, range_max] 内
        valid_mask = np.isfinite(ranges) & (ranges >= self.range_min) & (ranges <= self.range_max)
        if not np.any(valid_mask):
            return  # 无有效点，直接退出

        # 极坐标 → 直角坐标（FLU: Forward-Left-Up）
        xs = ranges[valid_mask] * np.cos(angles[valid_mask])
        ys = ranges[valid_mask] * np.sin(angles[valid_mask])
        points_flu = np.stack([xs, ys, np.zeros_like(xs)], axis=1)  # shape: (N, 3)

        # 欧氏聚类：将邻近点归为一类
        clusters = self.euclidean_clustering(points_flu, self.cluster_distance_threshold, self.min_cluster_size)

        # 使用 LiDAR 时间戳（更准确！）
        now = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9

        obstacles_enu = []  # 存储当前帧所有障碍物 {pos, t}

        for cluster in clusters:
            # 计算聚类中心（FLU）
            centroid_flu = np.mean(cluster, axis=0)
             # LiDAR 偏移和无人机中心高度（FLU 坐标系）
            lidar_offset = np.array([0.12, 0.0, 0.26])
            vehicle_center_height = np.array([0.0, 0.0, 0.24])
            centroid_flu = centroid_flu + lidar_offset + vehicle_center_height
            # FLU → FRD：y 和 z 取反（LiDAR 通常安装为 FLU，但 PX4 使用 FRD）
            centroid_frd = self.flu_to_frd(centroid_flu)
            # 若无位姿信息，跳过
            if self.latest_att_q is None or self.latest_localpos is None:
                continue

            # 将 FRD 点旋转到 NED（使用机体姿态）
            r_ned = self.rotate_point_by_quaternion(centroid_frd, self.latest_att_q)
            # 无人机在 NED 中的位置
            veh_pos_ned = np.array([self.latest_localpos.x,
                                    self.latest_localpos.y,
                                    self.latest_localpos.z])
            # 障碍物在 NED 中的位置 = 无人机位置 + 旋转后的相对向量
            obstacle_pos_ned = veh_pos_ned + r_ned
            # NED → ENU：x<->y, z取反（ENU: East-North-Up）
            obstacle_pos_enu = self.ned_to_enu(obstacle_pos_ned)

            # 保存障碍物（ENU 位置 + 时间戳）
            obstacles_enu.append({'pos': obstacle_pos_enu, 't': now})

        # 更新当前帧障碍物（供定时器使用）
        self.curr_obstacles = obstacles_enu


    # ---------------- 定时器任务：每0.5秒执行一次 ----------------
    def track_and_compute_speed(self):
        """
        核心逻辑：障碍物跟踪 + 速度估计
        - 使用历史轨迹窗口（默认3秒）计算原始速度
        - 用一阶低通滤波平滑速度
        - 匹配障碍物 ID（基于最近邻）
        - 清理超时障碍物
        - 发布结果
        """
        # 初始化历史记录字典（首次调用时创建）
        if not hasattr(self, 'obstacle_histories'):
            self.obstacle_histories = {}      # {id: [(pos, t), ...]}
            self.filtered_velocities = {}     # {id: [vx, vy, vz]}
            self.speed_window = getattr(self, 'speed_window', 2.0)      # 速度计算窗口（秒）
            self.lpf_alpha = getattr(self, 'lpf_alpha', 0.4)            # 低通滤波系数（0~1）
            self.obstacle_timeout = getattr(self, 'obstacle_timeout', 3.0)  # 障碍物超时时间（秒）

        obs_array = []  # 用于发布：[x,y,z,vx,vy,vz,...]
        assigned_prev_ids = set()  # 本帧已匹配的障碍物 ID

        # 若当前无障碍物，直接退出
        if not hasattr(self, 'curr_obstacles') or len(self.curr_obstacles) == 0:
            return

        # 获取当前帧最大时间戳（作为“当前时间”参考）
        t_max = max(float(obs['t']) for obs in self.curr_obstacles)

        # 遍历当前帧每个障碍物
        for obs in self.curr_obstacles:
            pos = np.array(obs['pos'], dtype=float)
            t = float(obs['t'])

            # ========== 1. 障碍物 ID 匹配 ==========
            matched_id, prev = self.find_matching_prev_obstacle(pos)
            if matched_id is None:
                # 未匹配到 → 新障碍物
                matched_id = self.next_obs_id
                self.next_obs_id += 1
                self.obstacle_histories[matched_id] = []
                self.filtered_velocities[matched_id] = np.zeros(3, dtype=float)
                self.prev_obstacles[matched_id] = {'pos': pos.copy(), 't': t}

            # ========== 2. 更新轨迹历史 ==========
            hist = self.obstacle_histories.setdefault(matched_id, [])
            hist.append((pos.copy(), t))

            # 移除超出速度窗口的历史点（滑动窗口）
            cutoff = t - self.speed_window
            while len(hist) > 0 and hist[0][1] < cutoff:
                hist.pop(0)

            # ========== 3. 原始速度估计（窗口首尾差分） ==========
            vel_raw = np.zeros(3, dtype=float)
            if len(hist) >= 2:
                pos_old, t_old = hist[0]      # 窗口最早点
                pos_new, t_new = hist[-1]     # 窗口最新点
                dt = max(t_new - t_old, 1e-6) # 防除零
                vel_raw = (pos_new - pos_old) / dt

            # ========== 4. 一阶低通滤波 ==========
            v_prev = self.filtered_velocities.get(matched_id, np.zeros(3, dtype=float))
            alpha = float(self.lpf_alpha)
            vel_filtered = alpha * vel_raw + (1.0 - alpha) * v_prev
            self.filtered_velocities[matched_id] = vel_filtered

            # ========== 5. 更新 prev_obstacles（用于下次匹配） ==========
            self.prev_obstacles[matched_id] = {'pos': pos.copy(), 't': t}

            # ========== 6. 准备发布数据 ==========
            obs_array.extend([
                float(pos[0]), float(pos[1]), float(pos[2]),
                float(vel_filtered[0]), float(vel_filtered[1]), float(vel_filtered[2])
            ])
            assigned_prev_ids.add(matched_id)

            # 打印日志
            speed_filtered = float(np.linalg.norm(vel_filtered))
            speed_raw = float(np.linalg.norm(vel_raw))
            self.get_logger().info(
                f"[Obstacle {matched_id}] ENU pos={pos.tolist()}, "
                f"raw_speed={speed_raw:.3f} m/s, filtered_speed={speed_filtered:.3f} m/s"
            )
            print(f"[Obstacle {matched_id}] ENU pos={pos.tolist()}, "
                f"raw_speed={speed_raw:.3f} m/s, filtered_speed={speed_filtered:.3f} m/s")

        # ========== 7. 清理超时障碍物 ==========
        stale_ids = []
        for pid, info in list(self.prev_obstacles.items()):
            last_t = float(info.get('t', 0.0))
            # 若该障碍物在本帧未被匹配，且超时 → 删除
            if (t_max - last_t) > float(self.obstacle_timeout) and (pid not in assigned_prev_ids):
                stale_ids.append(pid)

        for sid in stale_ids:
            self.prev_obstacles.pop(sid, None)
            self.obstacle_histories.pop(sid, None)
            self.filtered_velocities.pop(sid, None)
            self.get_logger().debug(f"Removed stale obstacle id={sid} (last seen {t_max - last_t:.2f}s ago)")

        # ========== 8. 发布结果 ==========
        msg = Float32MultiArray()
        msg.data = obs_array
        self.obs_pub.publish(msg)


    # ---------------- 工具函数 ----------------

    def euclidean_clustering(self, points, tol, min_pts):
        """
        简易欧氏聚类（适用于小规模点云）
        - 输入：points (N,3)
        - 输出：[cluster1, cluster2, ...]，每个 cluster 是 (M,3) 数组
        - 算法：BFS 遍历邻接点
        """
        if points.shape[0] == 0:
            return []
        N = points.shape[0]
        visited = np.zeros(N, dtype=bool)
        clusters = []
        # 预计算所有点对距离（O(N^2)，仅适用于小点云）
        dists = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)
        for i in range(N):
            if visited[i]:
                continue
            queue = [i]
            comp = []
            visited[i] = True
            while queue:
                idx = queue.pop()
                comp.append(idx)
                # 找出所有距离 <= tol 的邻居
                neighs = np.where(dists[idx] <= tol)[0]
                for n in neighs:
                    if not visited[n]:
                        visited[n] = True
                        queue.append(n)
            # 只保留足够大的簇
            if len(comp) >= min_pts:
                clusters.append(points[comp])
        return clusters

    def flu_to_frd(self, p_flu):
        """FLU (Forward-Left-Up) → FRD (Forward-Right-Down)"""
        return np.array([p_flu[0], -p_flu[1], -p_flu[2]])

    def rotate_point_by_quaternion(self, v, q):
        """
        使用四元数旋转点（q = [w, x, y, z]）
        手动构建旋转矩阵（避免依赖 tf_transformations）
        """
        w, x, y, z = q
        R = np.array([
            [1 - 2 * (y*y + z*z), 2 * (x*y - z*w),     2 * (x*z + y*w)],
            [2 * (x*y + z*w),     1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w),     2 * (y*z + x*w),     1 - 2 * (x*x + y*y)]
        ])
        return R @ v

    def ned_to_enu(self, p_ned):
        """NED (North-East-Down) → ENU (East-North-Up)"""
        return np.array([p_ned[1], p_ned[0], -p_ned[2]])

    def find_matching_prev_obstacle(self, pos):
        """
        最近邻匹配：在 self.prev_obstacles 中找距离最近的障碍物
        - 若距离 <= obs_match_radius，则匹配成功
        """
        best_id, best_dist = None, float('inf')
        for pid, info in self.prev_obstacles.items():
            d = np.linalg.norm(pos - info['pos'])
            if d < best_dist:
                best_dist = d
                best_id = pid
        if best_dist <= self.obs_match_radius:
            return best_id, self.prev_obstacles[best_id]
        return None, None


# ---------------- 主函数 ----------------
def main(args=None):
    rclpy.init(args=args)
    node = LidarPX4Processor()
    try:
        rclpy.spin(node)  # 阻塞运行，处理回调
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
