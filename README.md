# PX4-ROS2-2D-LiDAR-Obstacle-Avoidance

A lightweight ROS 2 node for real-time 2D LiDAR-based obstacle detection and tracking on PX4-powered drones.

## 📌 Overview

This package enables autonomous drones to perceive and track nearby obstacles using a 2D LiDAR (e.g., RPLIDAR, Hokuyo) in cluttered environments such as indoors, forests, or urban canyons. It integrates with PX4 via the ROS 2 microRTPS bridge to fuse LiDAR scans with vehicle state (position and attitude), transforming obstacle points into the global ENU frame and publishing their positions and velocities for use by path planners or avoidance controllers.

## ✨ Features

- Real-time LiDAR point filtering and Euclidean clustering  
- Coordinate transformation: LiDAR → drone body → NED → global ENU  
- Obstacle tracking across scans using nearest-neighbor association  
- Velocity estimation with configurable time window and low-pass filtering  
- Minimal dependencies — no `tf2`, pure ROS 2 + PX4 messages  
- Optimized for embedded platforms (Jetson, Raspberry Pi, etc.)

## 📦 Requirements

- **ROS 2** (Humble or later recommended)  
- **PX4 Autopilot** with running  
- 2D LiDAR publishing `/laser_scan` (`sensor_msgs/LaserScan`)  
- PX4 topics:  
  - `/fmu/out/vehicle_local_position`  
  - `/fmu/out/vehicle_attitude`

4. View output:
   ```bash
   ros2 topic echo /obstacles_enu
   ```

## 📡 Output

- **`/obstacles_enu`** (`std_msgs/Float32MultiArray`)  
  Flattened list of `[x1, y1, vx1, vy1, x2, y2, vx2, vy2, ...]` for all tracked obstacles in ENU coordinates.

## 🔧 Parameters (configurable via YAML or CLI)

| Parameter | Default | Description |
|----------|--------|-------------|
| `detection_range_max` | 5.0 | Max LiDAR detection range (m) |
| `cluster_tolerance` | 0.3 | Distance threshold for clustering (m) |
| `min_cluster_size` | 3 | Min points per obstacle |
| `tracking_radius` | 0.8 | Max distance to associate obstacles across frames (m) |
| `speed_window` | 1.0 | Time window for velocity estimation (s) |
| `lpf_alpha` | 0.5 | Low-pass filter coefficient (0–1) |

## 🛠️ Notes

- Assumes LiDAR is **forward-facing** and mounted with **0° yaw** relative to the drone body.
- For best results, align LiDAR frame with drone FRD convention.
- Designed for **planar environments**; not suitable for 3D obstacle avoidance.

