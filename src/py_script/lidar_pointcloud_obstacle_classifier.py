#!/usr/bin/env python3
"""
基于 3D LiDAR 点云(/lidar_points, sensor_msgs/PointCloud2)的障碍物聚类与分类节点。

功能：
1) 对点云进行欧氏聚类。
2) 通过几何形状区分墙体(wall)与柱体(pillar)。
3) 通过跨帧跟踪估计速度，区分静态/动态障碍物。
4) 将障碍物统一放入容器，同时将静态柱体与动态柱体分开存储，便于直接提取位置与速度。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


@dataclass
class Obstacle:
    """单个障碍物结构化数据。"""

    obstacle_id: int
    kind: str              # wall / pillar
    motion_state: str      # static / dynamic
    position: np.ndarray   # [x, y, z]
    velocity: np.ndarray   # [vx, vy, vz]
    size_xyz: np.ndarray   # 包围盒尺寸 [sx, sy, sz]


class PointCloudObstacleClassifier(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_obstacle_classifier')

        # ---- 参数 ----
        self.cluster_tolerance = 0.8
        self.min_cluster_size = 10
        self.max_range = 35.0
        self.tracking_radius = 1.2
        self.dynamic_speed_threshold = 0.25  # m/s
        self.wall_aspect_ratio_threshold = 3.0

        # ---- 订阅 ----
        self.points_sub = self.create_subscription(
            PointCloud2,
            '/lidar_points',
            self.pointcloud_callback,
            10,
        )

        # ---- 跟踪状态 ----
        self.next_track_id: int = 0
        self.prev_tracks: Dict[int, Dict[str, np.ndarray | float | str]] = {}

        # ---- 数据容器 ----
        self.all_obstacles: List[Obstacle] = []
        self.static_obstacles: List[Obstacle] = []
        self.dynamic_obstacles: List[Obstacle] = []
        self.static_walls: List[Obstacle] = []
        self.static_pillars: List[Obstacle] = []
        self.dynamic_pillars: List[Obstacle] = []

        self.get_logger().info('PointCloud obstacle classifier initialized.')

    # ---------- 回调入口 ----------

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        points = self.read_valid_points(msg)
        if points.shape[0] == 0:
            self.clear_containers()
            return

        clusters = self.euclidean_clustering(
            points=points,
            tol=self.cluster_tolerance,
            min_pts=self.min_cluster_size,
        )

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # 每帧重建容器，保证“当前帧快照”语义
        self.clear_containers()
        matched_ids = set()

        for cluster in clusters:
            centroid = np.mean(cluster, axis=0)
            size_xyz = np.ptp(cluster, axis=0)  # max-min
            kind = self.classify_geometry(size_xyz)

            track_id, velocity = self.match_and_estimate_velocity(centroid, timestamp)
            speed = float(np.linalg.norm(velocity))
            motion_state = 'dynamic' if speed >= self.dynamic_speed_threshold else 'static'

            obs = Obstacle(
                obstacle_id=track_id,
                kind=kind,
                motion_state=motion_state,
                position=centroid,
                velocity=velocity,
                size_xyz=size_xyz,
            )
            self.push_to_containers(obs)
            matched_ids.add(track_id)

            self.prev_tracks[track_id] = {
                'position': centroid.copy(),
                'time': float(timestamp),
                'kind': kind,
                'velocity': velocity.copy(),
            }

        # 清理本帧未被匹配上的旧目标
        stale_ids = [tid for tid in self.prev_tracks.keys() if tid not in matched_ids]
        for tid in stale_ids:
            self.prev_tracks.pop(tid, None)

        self.log_frame_summary()

    # ---------- 核心流程 ----------

    def read_valid_points(self, msg: PointCloud2) -> np.ndarray:
        raw_points = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.array(list(raw_points), dtype=np.float32)
        if pts.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float32)

        d = np.linalg.norm(pts[:, :3], axis=1)
        valid = np.isfinite(d) & (d <= self.max_range)
        return pts[valid]

    def euclidean_clustering(self, points: np.ndarray, tol: float, min_pts: int) -> List[np.ndarray]:
        if points.shape[0] == 0:
            return []

        # 使用 DBSCAN 进行欧式空间聚类：
        # - eps 对应聚类半径阈值；
        # - min_samples 对应簇最小点数；
        # - label=-1 为噪声点。
        dbscan = DBSCAN(eps=tol, min_samples=min_pts, metric='euclidean')
        labels = dbscan.fit_predict(points)

        clusters: List[np.ndarray] = []
        unique_labels = [lab for lab in np.unique(labels) if lab != -1]
        for lab in unique_labels:
            cluster = points[labels == lab]
            if cluster.shape[0] >= min_pts:
                clusters.append(cluster)

        return clusters

    def classify_geometry(self, size_xyz: np.ndarray) -> str:
        """
        基于包围盒横向尺寸判断几何类型：
        - 墙体：长宽比明显更大；
        - 柱体：形状更接近“圆柱在平面投影”的紧凑簇。
        """
        sx = max(float(size_xyz[0]), 1e-4)
        sy = max(float(size_xyz[1]), 1e-4)
        major = max(sx, sy)
        minor = min(sx, sy)
        aspect = major / minor
        return 'wall' if aspect >= self.wall_aspect_ratio_threshold else 'pillar'

    def match_and_estimate_velocity(self, pos: np.ndarray, t: float) -> Tuple[int, np.ndarray]:
        best_id: Optional[int] = None
        best_dist = math.inf

        for track_id, info in self.prev_tracks.items():
            prev_pos = np.array(info['position'], dtype=float)
            dist = float(np.linalg.norm(pos - prev_pos))
            if dist < best_dist:
                best_dist = dist
                best_id = track_id

        if best_id is None or best_dist > self.tracking_radius:
            new_id = self.next_track_id
            self.next_track_id += 1
            return new_id, np.zeros(3, dtype=float)

        info = self.prev_tracks[best_id]
        prev_pos = np.array(info['position'], dtype=float)
        prev_t = float(info['time'])
        dt = max(t - prev_t, 1e-6)
        vel = (pos - prev_pos) / dt
        return best_id, vel

    # ---------- 容器管理 ----------

    def clear_containers(self) -> None:
        self.all_obstacles = []
        self.static_obstacles = []
        self.dynamic_obstacles = []
        self.static_walls = []
        self.static_pillars = []
        self.dynamic_pillars = []

    def push_to_containers(self, obs: Obstacle) -> None:
        self.all_obstacles.append(obs)

        if obs.motion_state == 'static':
            self.static_obstacles.append(obs)
        else:
            self.dynamic_obstacles.append(obs)

        if obs.kind == 'wall' and obs.motion_state == 'static':
            self.static_walls.append(obs)

        if obs.kind == 'pillar' and obs.motion_state == 'static':
            self.static_pillars.append(obs)

        if obs.kind == 'pillar' and obs.motion_state == 'dynamic':
            self.dynamic_pillars.append(obs)

    # ---------- 对外提取接口 ----------

    def get_static_pillar_states(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """返回静态柱体[(position, velocity), ...]。"""
        return [(o.position.copy(), o.velocity.copy()) for o in self.static_pillars]

    def get_dynamic_pillar_states(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """返回动态柱体[(position, velocity), ...]。"""
        return [(o.position.copy(), o.velocity.copy()) for o in self.dynamic_pillars]

    def log_frame_summary(self) -> None:
        self.get_logger().info(
            'clusters=%d | static=%d dynamic=%d | static_walls=%d static_pillars=%d dynamic_pillars=%d'
            % (
                len(self.all_obstacles),
                len(self.static_obstacles),
                len(self.dynamic_obstacles),
                len(self.static_walls),
                len(self.static_pillars),
                len(self.dynamic_pillars),
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudObstacleClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
