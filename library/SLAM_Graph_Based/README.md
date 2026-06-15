# Graph-Based SLAM

Thư viện Graph SLAM 2D nhẹ, viết bằng C++17 cho AGV robot ROS2.

## Tính năng

- SE(2) Pose Graph
- Ràng buộc Odometry
- Phát hiện Loop Closure
- Thuật toán Gauss-Newton để tối ưu hóa vị trí các pose của robot
- Xây dựng bản đồ Occupancy Grid
- Tương thích ROS2

## Yêu cầu
 - ROS2 Humble 22.04
 - Gazebo
 - Rviz

## Kiến trúc

```
Odometry ──► PoseGraph ──► Loop Closure ──► Gauss-Newton ──► Optimized Graph ──► Occupancy Grid
                                                                                        │
                                                                                        │
                                                                                        │
                                                                             ┌──────────┴──────────┐
                                                                             │                     │
                                                                             │                     │
                                                                             ▼                     ▼
                                                                         Frontier Based        A* Algorithm

```

## Cấu trúc thư mục

```
include/
├── slam_graph.h              # Điều phối chính
├── pose_graph.h              # Cấu trúc dữ liệu đồ thị
├── map_builder.h             # Xây dựng bản đồ lưới
├── gauss_newton_solver.h     # Bộ tối ưu hóa
├── loop_closure_detector.h   # Phát hiện đóng vòng lặp
├── jacobian.h                # Jacobian giải tích
└── matrix.h                  # Ma trận + bộ giải

cpp/
├── slam_graph.cpp
├── pose_graph.cpp
├── map_builder.cpp
├── gauss_newton_solver.cpp
├── loop_closure_detector.cpp
├── jacobian.cpp
└── matrix.cpp
```

## Cách sử dụng

### Build
```bash
# Terminal
ros2 launch agv_robot autonomous_slam.launch.py

# Terminal 2:
. install/setup.bash && ros2 run agv_robot robot_controller
```

### Hình ảnh

```cpp

```

## API

### Topic

| Topic | Kiểu dữ liệu | Chiều | Tần số | Mô tả |
|-------|-------------|-------|---------|-------|
| `/agv_scan` | `sensor_msgs::msg::LaserScan` | Subscribe | 10 Hz | Dữ liệu quét LiDAR đầu vào, được sử dụng để thêm node odometry mới, phát hiện Loop Closure và cập nhật bản đồ Occupancy Grid. |
| `/map` | `nav_msgs::msg::OccupancyGrid` | Publish | 2 Hz | Bản đồ Occupancy Grid được xây dựng từ dữ liệu LiDAR và pose đã tối ưu hóa bởi Graph SLAM. Sử dụng biểu diễn Log-Odds với thuật toán Bresenham ray-casting. |
| `/slam_robot/graph_nodes` | `geometry_msgs::msg::PoseArray` | Publish | 0.5 Hz | Mảng các pose của tất cả node trong Pose Graph |
| `/slam_robot/graph_edges` | `visualization_msgs::msg::MarkerArray` | Publish | 0.5 Hz | Các cạnh trong Pose Graph dưới dạng LINE_LIST. Cạnh odometry hiển thị màu xanh dương, cạnh loop closure hiển thị màu đỏ. |
| `/slam_robot/loop_closure_event` | `std_msgs::msg::String` | Publish | Khi phát hiện | Thông báo mỗi khi phát hiện một ràng buộc đóng Loop Closure. |
| `/slam_robot/scan_visualization` | `sensor_msgs::msg::LaserScan` | Publish | 5 Hz | Dữ liệu LaserScan được publish lại (skip frame). |

### Subscribe

| Topic | Kiểu dữ liệu | QoS | Callback | Mô tả |
|-------|-------------|-----|----------|-------|
| `/agv_scan` | `sensor_msgs::msg::LaserScan` | `SensorDataQoS()` | `scanCallback()` | Nhận dữ liệu LiDAR 10Hz. Lưu vào bộ đệm (cache) cho graphSLAMcall, cập nhật MapBuilder (occupancy grid), và log khoảng cách 8 hướng xung quanh robot. |

Ngoài ra, node `SlamRobot` còn sử dụng **TF2** để nhận các phép biến đổi:

| TF Frame | Chiều | Mô tả |
|----------|-------|-------|
| `odom` → `base_link` | Nhận (Lookup) | Lấy pose hiện tại của robot trong hệ tọa độ odometry. Được lookup tại `slamTimerCallback()` (5Hz) và `scanCallback()` (10Hz) để đồng bộ thời gian với timestamp của scan. |

### Publish

| Topic | Kiểu dữ liệu | QoS | Mô tả |
|-------|-------------|-----|-------|
| `/map` | `nav_msgs::msg::OccupancyGrid` | `transient_local` QoS (1) | Bản đồ occupancy grid được publish định kỳ 2Hz bởi `mapBuilderTimerCallback()`. Bản đồ được xây dựng lại từ các node đã tối ưu hóa. |
| `/slam_robot/graph_nodes` | `geometry_msgs::msg::PoseArray` | `transient_local` QoS (5) | Các node trong Pose Graph được publish định kỳ 0.5Hz bởi `graphVizTimerCallback()`. Mỗi node có pose (x, y, theta) và được gắn nhãn z=0.05 để hiển thị trên Rviz. |
| `/slam_robot/graph_edges` | `visualization_msgs::msg::MarkerArray` | `transient_local` QoS (5) | Các cạnh của Pose Graph được publish cùng lúc với graph_nodes. Gồm hai Marker: odom_edges (màu xanh dương, độ dày 0.02) và loop_edges (màu đỏ, độ dày 0.04). |
| `/slam_robot/loop_closure_event` | `std_msgs::msg::String` | `transient_local` QoS (5) | Publish ngay khi phát hiện loop closure mới trong `graphSLAMcall()`. |
| `/slam_robot/scan_visualization` | `sensor_msgs::msg::LaserScan` | `transient_local` QoS (10) | Publish scan visualization ở tần suất 5Hz (skip 1 frame) để giảm tải băng thông. |

Ngoài ra, node `SlamRobot` còn phát **TF2**:

| TF Frame | Chiều | Tần suất | Mô tả |
|----------|-------|---------|-------|
| `map` → `odom` | Phát (Broadcast) | 50 Hz | Phép biến đổi map→odom được hiệu chỉnh bởi Graph SLAM, cho phép hiển thị chính xác vị trí robot trên Rviz. Được broadcast bởi `tf_broadcast_timer_` ở 50Hz. |

### Sơ đồ luồng dữ liệu (Data Flow Diagram)

```
                         ┌─────────────────┐
                         │  /agv_scan      │ ◄──── LiDAR (10 Hz)
                         │  (LaserScan)    │
                         └────────┬────────┘
                                  │
                    ┌─────────────┴─────────────┐
                    │     scanCallback()        │
                    │  • Lưu scan vào cache     │
                    │  • Cập nhật MapBuilder    │
                    │  • Log khoảng cách 8 hướng│
                    └─────────────┬─────────────┘
                                  │
              ┌───────────────────┼───────────────────┐
              │                   │                   │
              ▼                   ▼                   ▼
    ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
    │  cached_scan_   │  │  Odometry TF    │  │   MapBuilder    │
    │  ranges_        │  │  (odom→base)    │  │  (Occupancy     │
    │  (cache)        │  │                 │  │   Grid)         │
    └────────┬────────┘  └────────┬────────┘  └────────┬────────┘
             │                    │                    │
             └────────┬───────────┘                    │
                      │                                │
                      ▼                                │
         ┌───────────────────────┐                     │
         │   graphSLAMcall()     │                     │
         │   (5 Hz)              │                     │
         │                       │                     │
         │ 1. addOdometryNode()  │                     │
         │    → Thêm node mới    │                     │
         │ 2. addLoopClosures()  │                     │
         │    → Phát hiện LC     │                     │
         │ 3. optimizeIfNeeded() │                     │
         │    → Gauss-Newton     │                     │
         └──────────┬────────────┘                     │
                    │                                  │
          ┌─────────┴──────────┐                       │
          ▼                    ▼                       │
   ┌────────────┐      ┌──────────────┐                │
   │ /slam_robot│      │ map→odom TF  │                │
   │ /loop_     │      │ (Broadcast   │                │
   │ closure_   │      │  50 Hz)      │                │
   │ event      │      └──────────────┘                │
   └────────────┘                                      │
                                                       │
          ┌────────────────────────────────────────────┘
          ▼
   ┌──────────────────────┐
   │ /map (OccupancyGrid) │ ────► 2 Hz
   └──────────────────────┘

Visualization:
   ┌─────────────────────────────────────────────┐
   │  /slam_robot/graph_nodes (PoseArray)        │ ────► 0.5 Hz
   │  /slam_robot/graph_edges (MarkerArray)      │ ────► 0.5 Hz
   │  /slam_robot/scan_visualization (LaserScan) │ ────► 5 Hz
   │  /slam_robot/loop_closure_event             │
   └─────────────────────────────────────────────┘
```

## Tham số

| Tham số | Giá trị | Mô tả |
|---------|----------|-------|
| `min_travel_dist` | 0.30 m | Khoảng cách tối thiểu để thêm node mới |
| `min_travel_angle` | 0.15 rad | Góc quay tối thiểu để thêm node mới |
| `odom_omega_xy` | 50.0 | Trọng số odometry (x, y) |
| `odom_omega_theta` | 80.0 | Trọng số odometry (θ) |
| `gn_iterations` | 10 | Số lần lặp Gauss-Newton |
| `dist_threshold` | 2.0 m | Ngưỡng khoảng cách cho loop closure |
| `angle_threshold` | 1.2 rad | Ngưỡng góc cho loop closure |
| `correlation_threshold` | 0.80 | Ngưỡng tương quan scan |
| `min_node_gap` | 10 | Khoảng cách chỉ số node tối thiểu |
| `omega_xy` | 200.0 | Trọng số loop closure (x, y) |
| `omega_theta` | 400.0 | Trọng số loop closure (θ) |

## Tài liệu tham khảo

- Grisetti, G. et al. (2010). *A Tutorial on Graph-Based SLAM*. IEEE Intell. Transp. Syst. Mag.
- Thrun, S. et al. (2005). *Probabilistic Robotics*. MIT Press.
- Kümmerle, R. et al. (2011). *g²o: A General Framework for Graph Optimization*. IEEE ICRA.