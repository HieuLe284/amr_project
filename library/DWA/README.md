# Dynamic Window Approach (DWA) Local Planner

Thư viện DWA (Dynamic Window Approach) 2D nhẹ, viết bằng C++17 cho robot ROS2, dựa trên thuật toán Fox, Burgard & Thrun (1997).

## Tính năng

- Dynamic Window — tính không gian vận tốc khả thi từ giới hạn gia tốc
- Grid Sampling — duyệt mẫu (v, w) trong Dynamic Window
- Exact Circular Curvature — tính khoảng cách va chạm chính xác trên quỹ đạo tròn
- Objective Function — G(v,w) = α·heading + β·clearance + γ·velocity
- Admissible Velocities — kiểm tra khả năng phanh trước va chạm
- Lookahead Goal — tự động tìm waypoint mục tiêu từ A* path
- Tích hợp SLAM + Frontier-Based Exploration + A* Global Planner
- Tương thích ROS2

## Yêu cầu

- ROS2 Humble 22.04
- Gazebo
- Rviz
- SLAM
- Frontier
- A*

## Kiến trúc

```
Robot State ──► DynamicWindow ──► Grid Sampling ──► TrajectoryScorer ──► (v*, w*)
 (v, w, θ)      (Vd ∩ Vs)        (v_i, w_j)        (G = α·h + β·c + γ·v)     │
     ▲                                                                       │
     │                                                                       ▼
     └──────────────────── ← ← ← ─── /cmd_vel ─────────────────────── Robot
```

### Thuật toán DWA — Quy trình mỗi chu kỳ điều khiển (5 Hz)

```
Input:  s_cur = (x, y, θ, v, w)     — trạng thái robot hiện tại
        scan_ranges                  — dữ liệu LiDAR thô
        path                         — A* path (vector waypoints world frame)

Bước 1 — Dynamic Window (Vd):
    Vd = { (v,w) | v ∈ [v_cur - a_v·dt,  v_cur + a_v·dt]
                   w ∈ [w_cur - a_w·dt,  w_cur + a_w·dt] }

Bước 2 — Kinematic Limits (Vs):
    Vs = { (v,w) | v ∈ [v_min, v_max],  w ∈ [-w_max, w_max] }

Bước 3 — Feasible Window (Vf = Vd ∩ Vs):
    [v_low, v_high] × [w_low, w_high]

Bước 4 — Scan → Obstacles (robot frame):
    (x_obs, y_obs) = (d · cos(angle), d · sin(angle))

Bước 5 — Lookahead Goal:
    Từ A* path → tìm waypoint cách robot ≥ lookahead_dist
    Tính goal_angle trong robot frame

Bước 6 — Grid Sampling:
    Duyệt (v_i, w_j) trong Vf với bước:
        dv = (v_high - v_low) / v_samples
        dw = (w_high - w_low) / w_samples

Bước 7 — Admissible Velocities (Va):
    Với mỗi (v, w), tính khoảng cách an toàn dist
    Chỉ giữ nếu:  v ≤ √(2 · dist · v_dot_b)
                và w ≤ √(2 · dist · w_dot_b)

Bước 8 — Objective Function:
    G(v, w) = α · heading(v,w) + β · clearance(v,w) + γ · velocity(v,w)

Output: (v*, w*) — vận tốc tối ưu
```

## Cấu trúc thư mục

```
include/
├── dwa_config.h             # Tham số cấu hình (kinematic, sampling, weights)
├── dwa_state.h              # Trạng thái robot (x, y, θ, v, w)
├── dwa_window.h             # Dynamic Window (Vd ∩ Vs)
├── dwa_scoring.h            # Hàm mục tiêu G(v,w) + khoảng cách va chạm
└── dwa_planner.h            # Điều phối chính

cpp/
├── dwa_window.cpp
├── dwa_scoring.cpp
└── dwa_planner.cpp
```

## API

### Thuật toán (Algorithm)

#### Dynamic Window (Vd)

Không gian vận tốc mà robot có thể đạt được trong 1 bước thời gian dt:

| Thành phần | Công thức |
|-----------|-----------|
| Vận tốc tịnh tiến | `v ∈ [v_cur - a_v·dt, v_cur + a_v·dt]` |
| Vận tốc góc | `w ∈ [w_cur - a_w·dt, w_cur + a_w·dt]` |

Giao với không gian vận tốc tĩnh học (Vs):

| Thành phần | Công thức |
|-----------|-----------|
| Vận tốc tịnh tiến | `v ∈ [v_min, v_max]` |
| Vận tốc góc | `w ∈ [-w_max, w_max]` |

→ **Không gian khả thi:** `Vf = Vd ∩ Vs`

#### Admissible Velocities (Va)

Kiểm tra robot có thể phanh kịp trước khi va chạm:

| Công thức | Mô tả |
|-----------|-------|
| `v ≤ √(2 · dist · v_dot_b)` | Vận tốc tịnh tiến phải đủ nhỏ để phanh kịp |
| `w ≤ √(2 · dist · w_dot_b)` | Vận tốc góc phải đủ nhỏ để phanh kịp |

Trong đó:
- `dist` — khoảng cách di chuyển dọc quỹ đạo trước khi va chạm [m]
- `v_dot_b` — gia tốc phanh tịnh tiến [m/s²]
- `w_dot_b` — gia tốc phanh góc [rad/s²]

#### Objective Function

```
G(v, w) = α · heading(v,w) + β · clearance(v,w) + γ · velocity(v,w)
```

| Thành phần | Công thức | Ý nghĩa |
|-----------|-----------|---------|
| **heading** | `1 - |θ_pred - θ_goal| / π` | Hướng tới mục tiêu (θ_pred: góc dự đoán sau khi phanh, θ_goal: góc từ robot đến lookahead waypoint) |
| **clearance** | `dist / D_normalize` | Khoảng cách an toàn đến vật cản gần nhất (đã chuẩn hóa) |
| **velocity** | `|v| / v_max` | Khuyến khích tốc độ cao |

Trọng số mặc định: `α = 0.55`, `β = 0.25`, `γ = 0.20`

#### Khoảng cách va chạm (Exact Circular Curvature)

Với quỹ đạo tròn bán kính `r = v / w`:
- Tính chính xác độ dài cung tròn đến khi mép robot (r + robot_radius) chạm vật cản
- Nếu `w ≈ 0` (đi thẳng): khoảng cách là khoảng cách đến vật cản trên đường thẳng

#### Lookahead Goal

Từ A* path, DWA tự động tìm waypoint mục tiêu:

1. Duyệt từ waypoint đầu tiên trong path
2. Tìm waypoint đầu tiên cách robot ≥ `LOOKAHEAD_DIST` (0.4 m)
3. Nếu không có → lấy waypoint cuối cùng (goal)
4. Tính `goal_angle` = góc từ robot → waypoint đó (trong robot frame)

### Topic 
| Topic | Kiểu dữ liệu | Chiều | Tần suất | Mô tả |
|-------|-------------|-------|---------|-------|
| `/agv_scan` | `sensor_msgs::msg::LaserScan` | Subscribe | 10 Hz | Dữ liệu LiDAR đầu vào. DWA chuyển đổi scan → danh sách vật cản (x, y) trong robot frame để tính clearance và admissible velocities. |
| `/cmd_vel` | `geometry_msgs::msg::Twist` | Publish | 5 Hz | Lệnh vận tốc điều khiển robot (linear.x, angular.z). Được tính bởi DWA computeVelocity() hoặc Stuck Recovery khi robot bị kẹt. |

### Subscribe 

| Topic | Kiểu dữ liệu | QoS | Callback | Mô tả |
|-------|-------------|-----|----------|-------|
| `/agv_scan` | `sensor_msgs::msg::LaserScan` | `SensorDataQoS()` | `scanCallback()` | Nhận dữ liệu LiDAR 10Hz. Lưu vào bộ đệm (cached_scan_ranges_) cho DWA local planner. DWA chuyển scan thành danh sách vật cản trong robot frame để tính khoảng cách an toàn. |

Ngoài ra, DWA còn sử dụng **TF2** thông qua node chủ và **A* path**:

| TF Frame / Dữ liệu | Chiều | Mô tả |
|--------------------|-------|-------|
| `map` → `base_link` | Nhận (Lookup) | Lấy pose robot (x, y, θ) trong world frame để tính lookahead goal từ A* path. |
| A* `cached_global_path_` | Nội bộ | Nhận danh sách waypoints từ A* global planner để DWA tự động tìm waypoint lookahead và tính goal_angle. |

### Publish 

| Topic | Kiểu dữ liệu | QoS | Mô tả |
|-------|-------------|-----|-------|
| `/cmd_vel` | `geometry_msgs::msg::Twist` | QoS (10) | Lệnh vận tốc điều khiển robot: `linear.x = v*` [m/s], `angular.z = w*` [rad/s]. Được publish bởi `localPlannerTimerCallback()` (5Hz) hoặc Stuck Recovery override khi robot bị kẹt. |

### Sơ đồ luồng dữ liệu (Data Flow Diagram)

```
                        ┌──────────────────────────┐
                        │     SLAM Robot Node       │
                        │  (slam_robot)             │
                        └──────────────────────────┘

                        ┌──────────────────────────┐
                        │  Frontier-Based + A*      │
                        │  (globalPlannerTimerCallback) │
                        │  → cached_global_path_    │
                        └────────────┬─────────────┘
                                     │ A* waypoints
                                     ▼
┌────────────────────────────────────────────────────────────────────┐
│                  AutonomousExploration Node                        │
│                                                                    │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │            localPlannerTimerCallback (5 Hz — 200ms)         │  │
│  │                                                            │  │
│  │  Bước 0 — Kiểm tra điều kiện:                              │  │
│  │    • map_initialized_?                                      │  │
│  │    • exploration_mode_?                                     │  │
│  │    • cached_path_valid_? (nếu không → publish stop)         │  │
│  │                                                            │  │
│  │  Bước 1 — Lấy pose robot từ TF (map→base_link)             │  │
│  │    → rx, ry, rth, current_v_, current_w_                   │  │
│  │                                                            │  │
│  │  Bước 2 — Stuck Detection:                                 │  │
│  │    dist_moved = hypot(rx - prev_x, ry - prev_y)            │  │
│  │    Nếu đang có lệnh (|v|>0.01 || |w|>0.02)                 │  │
│  │    mà dist_moved < 0.03m trong 1.5s                        │  │
│  │      → kích hoạt Stuck Recovery (override cmd_vel)         │  │
│  │      Phase 1: Lùi 0.6s (v=-0.06)                          │  │
│  │      Phase 2: Xoay 1.0s (ω=±0.8, chọn hướng clearance lớn) │  │
│  │      Phase 3: Chờ 0.5s → resume DWA                       │  │
│  │                                                            │  │
│  │  Bước 3 — DWA computeVelocity():                          │  │
│  │    ┌────────────────────────────────────────────────────┐  │  │
│  │    │  a. DynamicWindow::compute(state, config)           │  │  │
│  │    │     Vd ∩ Vs → [v_low, v_high] × [w_low, w_high]   │  │  │
│  │    │                                                    │  │  │
│  │    │  b. scanToObstacles(ranges, angle_min, inc, yaw)   │  │  │
│  │    │     → vector<pair<double,double>> obstacles        │  │  │
│  │    │       (x_obs, y_obs trong robot frame)             │  │  │
│  │    │                                                    │  │  │
│  │    │  c. findLookaheadGoal(robot_x, robot_y, θ, path)   │  │  │
│  │    │     → goal_angle (góc đến waypoint trong robot frame)│  │  │
│  │    │                                                    │  │  │
│  │    │  d. Grid Sampling + Scoring:                       │  │  │
│  │    │     Với mỗi (v_i, w_j) trong grid:                 │  │  │
│  │    │       • computeDistance(v, w, obstacles, config)   │  │  │
│  │    │         → dist (khoảng cách an toàn)               │  │  │
│  │    │       • Admissible check:                          │  │  │
│  │    │         v ≤ √(2·dist·v_dot_b) &&                   │  │  │
│  │    │         w ≤ √(2·dist·w_dot_b)                     │  │  │
│  │    │       • score(state, v, w, dist, goal_angle, cfg)  │  │  │
│  │    │         G = α·heading + β·clearance + γ·velocity   │  │  │
│  │    │       • Chọn (v*, w*) có G lớn nhất               │  │  │
│  │    └────────────────────────────────────────────────────┘  │  │
│  │                                                            │  │
│  │  Bước 4 — Publish /cmd_vel:                               │  │
│  │    cmd.linear.x = v_star                                   │  │
│  │    cmd.angular.z = w_star                                  │  │
│  │    pub_cmd_->publish(cmd)                                  │  │
│  └──────────────────────────┬──────────────────────────────────┘  │
│                             │ /cmd_vel (Twist)                    │
│                             ▼                                     │
│                      ┌─────────────┐                              │
│                      │   Robot     │                              │
│                      └─────────────┘                              │
└────────────────────────────────────────────────────────────────────┘
```

### Stuck Detection & Recovery

Cơ chế phát hiện kẹt khi DWA không thể di chuyển:

| Phase | Hành động | Thời gian | Vận tốc | Mô tả |
|-------|-----------|-----------|---------|-------|
| 0 | Bình thường | — | DWA | Robot hoạt động bình thường với DWA |
| 1 | **Lùi** | 0.6 s | `v = -0.06 m/s` | Lùi nhẹ để tạo khoảng trống phía trước |
| 2 | **Xoay** | 1.0 s | `ω = ±0.8 rad/s` | Xoay về hướng có clearance lớn nhất (dùng LiDAR so sánh left vs right clearance) |
| 3 | **Chờ** | 0.5 s | `v = 0, ω = 0` | Chờ hồi phục, cho DWA hoạt động lại |

**Ngưỡng phát hiện:** Robot có lệnh di chuyển (|v| > 0.01 || |w| > 0.02) nhưng **không di chuyển > 0.03m trong > 1.5s**.

## Tham số
| Tham số | Giá trị | Mô tả |
|---------|----------|-------|
| `v_max` | 0.30 m/s | Vận tốc tịnh tiến tối đa |
| `v_min` | -0.10 m/s | Vận tốc tịnh tiến tối thiểu (lùi) |
| `w_max` | 2.1 rad/s | Vận tốc góc tối đa |
| `a_v_max` | 3.0 m/s² | Gia tốc tịnh tiến tối đa |
| `a_w_max` | 7.0 rad/s² | Gia tốc góc tối đa |
| `v_dot_b` | 1.5 m/s² | Gia tốc phanh tịnh tiến |
| `w_dot_b` | 3.0 rad/s² | Gia tốc phanh góc |
| `robot_radius` | 0.15 m | Bán kính an toàn của robot |
| `dt` | 0.10 s | Bước thời gian mô phỏng Dynamic Window |
| `v_samples` | 30 | Số mẫu vận tốc tịnh tiến |
| `w_samples` | 50 | Số mẫu vận tốc góc |
| `alpha` | 0.55 | Trọng số heading (hướng tới mục tiêu) |
| `beta` | 0.25 | Trọng số clearance (khoảng cách vật cản) |
| `gamma` | 0.20 | Trọng số velocity (khuyến khích tốc độ) |
| `CROSS_TRACK_THRESH` | 0.20 m | Ngưỡng sai số lệch đường (bẻ lái về nếu lệch) |
| `LOOKAHEAD_DIST` | 0.4 m | Khoảng cách nhìn trước trên A* path |
| `LOOKAHEAD_CORR` | 1.0 | Hệ số hiệu chỉnh bẻ lái |
| `ESCAPE_TRIGGER_DIST` | 0.25 m | Ngưỡng kích hoạt thoát hiểm |
| `sensor_max_range` | 8.0 m | Tầm xa tối đa LiDAR |
| `D_normalize` | 2.5 m | Khoảng cách chuẩn hóa clearance (robot lo ngại vật cản trong 2.5m) |

## Tài liệu tham khảo

- Fox, D. et al. (1997). *The Dynamic Window Approach to Collision Avoidance*. IEEE Robot. Autom. Mag.
- Fox, D. et al. (2002). *A Dynamic Window Approach with Collision Prediction*. IEEE ICRA.
- Seder, M. & Petrovic, I. (2007). *Dynamic Window Based Approach to Mobile Robot Motion Control in the Presence of Moving Obstacles*. IEEE ICRA.