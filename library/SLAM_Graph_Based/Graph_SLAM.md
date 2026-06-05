## Luồng chạy của `slam_robot.cpp` — từ `main()` đến từng hàm

### 1. `main()` (dòng 310-318) — điểm khởi đầu

```
main()
  ├── rclcpp::init(argc, argv)           ← Khởi tạo ROS2
  ├── auto node = std::make_shared<SlamRobot>();  ← GỌI HÀM CONSTRUCTOR
  ├── executor = MultiThreadedExecutor    ← Executor đa luồng
  │     ├── executor.add_node(node)       ← Thêm node vào executor
  │     └── executor.spin()               ← Bắt đầu vòng lặp xử lý callback
  └── rclcpp::shutdown()
```

---

### 2. Constructor `SlamRobot()` (dòng 33-94)

Khi `make_shared<SlamRobot>()` chạy, constructor thực hiện lần lượt:

```
SlamRobot()
  ├── Tạo callback groups (lidar + slam)
  ├── Tạo TF2: tf_buffer_, tf_listener_, tf_broadcaster_
  ├── Tạo các Publishers:
  │     ├── /cmd_vel
  │     ├── /map              ← OccupancyGrid (transient_local QoS)
  │     ├── /slam_robot/graph_nodes      ← PoseArray
  │     ├── /slam_robot/graph_edges      ← MarkerArray
  │     ├── /slam_robot/loop_closure_event
  │     └── /slam_robot/scan_visualization
  ├── Tạo MapBuilder(0.05, 640, 400, -17, -10)
  │     └── map_builder_.setPublisher(pub_map_)
  ├── slam_graph_.setMapBuilder(&map_builder_)  ← Kết nối SlamGraph với MapBuilder
  ├── slam_graph_.init()                        ← Khởi tạo graph với 1 node anchor (0,0,0)
  ├── Tạo Subscriber: /agv_scan
  │     └── callback_group = lidar_
  ├── Tạo 2 timers:
  │     ├── timer_ (200ms = 5Hz) → gọi slamTimerCallback()
  │     └── map_timer_ (200ms = 5Hz) → gọi mapBuilderTimerCallback()
  └── Log thông báo khởi tạo xong
```

Sau constructor, executor bắt đầu spin → các callback chạy khi có sự kiện.

---

### 3. Luồng chính #1 — `scanCallback()` (dòng 253-305)

Đây là callback **chạy mỗi khi có dữ liệu LiDAR** từ topic `/agv_scan`:

```
scanCallback(scan_msg)
  │
  ├── broadcastMapOdomTF()     ← Broadcast TF map→odom (identity, 0,0,0)
  │
  ├── Cache LiDAR data:
  │     ├── cached_scan_ranges_ = scan->ranges
  │     ├── cached_scan_angle_min_ = scan->angle_min
  │     └── cached_scan_angle_increment_ = scan->angle_increment
  │
  ├── Tính và log 8 hướng khoảng cách (Front, Left, Right...)
  │
  ├── pub_scan_visualization_->publish(*scan)     ← Publish scan lên topic để Rviz xem
  │
  ├── Mỗi 2 scan: gọi map_builder_.buildOccupancyGrid()
  │     └── Nếu map chưa init → set map_initialized_ = true
  │
  └── [ĐOẠN BẠN THÊM] — try/catch lookupTransform "map"→"base_link"
        └── map_builder_.updateFromRanges(...)  ← Cập nhật occupancy grid từ LiDAR
```

---

### 4. Luồng chính #2 — `slamTimerCallback()` (dòng 219-240)

Callback này chạy **5Hz** (timer 200ms) — đây là core của SLAM:

```
slamTimerCallback()
  │
  ├── broadcastMapOdomTF()     ← Broadcast TF map→odom (identity)
  │
  ├── try: lookupTransform("map", "base_link")
  │     ├── x = tf.transform.translation.x
  │     ├── y = tf.transform.translation.y
  │     └── theta = từ quaternion → getRPY → yaw
  │
  ├── Nếu có TF → gọi graphSLAMcall(x, y, theta)  ← QUAN TRỌNG NHẤT
  │
  └── Nếu không có TF → log warning mỗi 5 giây và return
```

---

### 5. Luồng chính #3 — `graphSLAMcall(x, y, theta)` (dòng 117-214)

Đây là **hàm SLAM chính** — thực hiện 4 bước Graph-Based SLAM:

```
graphSLAMcall(x, y, theta)
  │
  ├── Bước 1 — FRONT-END (Odometry):
  │     └── slam_graph_.addOdometryNode(x, y, theta, scan_ranges, ...)
  │           ├── So sánh với node trước: nếu di chuyển < 0.3m && < 0.15 rad → return -1
  │           ├── Thêm node mới vào pose_graph.nodes
  │           ├── Tính δ body-frame: zx, zy, zt = R_prev^T · (t_new - t_prev)
  │           └── Thêm edge (i-1→i) với information matrix Ω_odom
  │
  ├── Nếu new_idx < 0 (chưa đủ di chuyển) → RETURN NGAY
  │
  ├── Bước 2 — LOOP CLOSURE:
  │     └── slam_graph_.addLoopClosures(new_idx)
  │           ├── Duyệt các node cũ, tính scan correlation
  │           ├── Nếu match → thêm edge loop-closure với Ω_loop cao hơn
  │           └── Trả về matched_id hoặc -1
  │
  ├── Bước 3 — BACK-END (Gauss-Newton):
  │     └── slam_graph_.optimizeIfNeeded()
  │           ├── Chỉ chạy nếu vừa phát hiện loop closure (new_loop_this_step_)
  │           ├── Giải hệ H·Δξ = -b → cập nhật tất cả poses
  │           └── Trả về true nếu đã optimization
  │
  ├── Bước 4 — MAP REBUILD (chỉ khi optimized == true):
  │     ├── slam_graph_.clearMap()          ← Xóa toàn bộ occupancy grid
  │     └── for i = 1..N:
  │           └── slam_graph_.updateMapFromNode(i)
  │                 └── map_builder_.updateFromRanges() ← Ray-cast từ node pose
  │
  └── Publish visualization:
        ├── /slam_robot/graph_nodes  (PoseArray — tất cả node poses)
        └── /slam_robot/graph_edges  (MarkerArray — đường nối các node)
```

---

### 6. Luồng phụ — `mapBuilderTimerCallback()` (dòng 245-248)

Callback 5Hz này **publish map** lên topic `/map`:

```
mapBuilderTimerCallback()
  └── Nếu map_initialized_ == true:
        └── map_builder_.publishMap(this->now())
              └── pub_map_->publish(buildOccupancyGrid(stamp))
```

Nó kiểm tra `map_initialized_` — biến này được set thành `true` ở `scanCallback()` dòng 284, nhưng chỉ khi `buildOccupancyGrid()` được gọi lần đầu tiên.

---

### 7. Sơ đồ thuật toán
```
------------------------------------------------- Graph SLAM -------------------------------------------------
                                      ┌───────────────┐
                                      │    BẮT ĐẦU    │
                                      └───────┬───────┘
                                              │
                                              ▼
                                      ┌───────────────────────────────┐
                                      │ Khởi tạo ROS2 Node            │
                                      │ - Subscriber /scan            │
                                      │ - TF Listener                 │
                                      │ - Pose Graph                  │
                                      │ - Loop Closure Detector       │
                                      │ - Optimizer                   │
                                      └───────┬───────────────────────┘
                                              │
                                              ▼
                                      ┌───────────────────────────────┐
                                      │ Chờ dữ liệu LiDAR             │
                                      └───────┬───────────────────────┘
                                              │
                                              ▼
                                      ┌───────────────────────────────┐
                                      │ Nhận LaserScan                │
                                      └───────┬───────────────────────┘
                                              │
                                              ▼
                                      ┌───────────────────────────────┐
                                      │ Lưu scan vào bộ nhớ đệm       │
                                      └───────┬───────────────────────┘
                                              │
                                              ▼
                                      ┌───────────────────────────────┐
                                      │ Lấy pose robot từ TF          │
                                      │ (x, y, yaw)                   │
                                      └───────┬───────────────────────┘
                                              │
                                              ▼
                                      ┌───────────────────────────────┐
                                      │ Đủ điều kiện tạo node mới ?   │
                                      └───────┬───────────────┬───────┘
                                              │Không           │Có
                                              │                │
                                              ▼                ▼
                                      ┌───────────────┐   ┌─────────────────────┐
                                      │ Quay lại chờ  │   │ Tạo node mới        │
                                      │ scan tiếp theo│   └─────────┬───────────┘
                                      └───────────────┘             │
                                                                    ▼
                                                          ┌─────────────────────┐
                                                          │ Tạo odometry edge   │
                                                          └─────────┬───────────┘
                                                                    │
                                                                    ▼
                                                          ┌─────────────────────┐
                                                          │ Phát hiện loop      │
                                                          │ closure             │
                                                          └─────────┬───────────┘
                                                                    │
                                                                    ▼
                                                      ┌──────────────────────────┐
                                                      │ Có loop closure không ?  │
                                                      └─────────┬─────────┬──────┘
                                                                │Không     │Có
                                                                │          │
                                                                ▼          ▼
                                                      ┌────────────────┐  ┌─────────────────────┐
                                                      │ Bỏ qua         │  │ Thêm loop edge      │
                                                      └───────┬────────┘  └─────────┬───────────┘
                                                              │                    │
                                                              └──────────┬─────────┘
                                                                          ▼
                                                          ┌──────────────────────────┐
                                                          │ Tối ưu hóa pose graph    │
                                                          │ (Gauss-Newton)           │
                                                          └─────────┬────────────────┘
                                                                    │
                                                                    ▼
                                                          ┌──────────────────────────┐
                                                          │ Cập nhật lại pose node   │
                                                          └─────────┬────────────────┘
                                                                    │
                                                                    ▼
                                                          ┌──────────────────────────┐
                                                          │ Xóa map cũ               │
                                                          └─────────┬────────────────┘
                                                                    │
                                                                    ▼
                                                          ┌──────────────────────────┐
                                                          │ Vẽ lại map từ graph      │
                                                          │ đã tối ưu                │
                                                          └─────────┬────────────────┘
                                                                    │
                                                                    ▼
                                                          ┌──────────────────────────┐
                                                          │ Publish Occupancy Grid   │
                                                          └─────────┬────────────────┘
                                                                    │
                                                                    ▼
                                                          ┌──────────────────────────┐
                                                          │ Quay lại chờ scan tiếp   │
                                                          └──────────────────────────┘
```
