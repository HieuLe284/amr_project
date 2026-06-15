# A* Global Planner

Thư viện A* (A-Star) Global Planner 2D nhẹ, viết bằng C++17 cho robot ROS2, dựa trên thuật toán Hart, Nilsson & Raphael (1968).

## Tính năng

- A* trên lưới 8-connectivity (8 hướng di chuyển: ngang, dọc, chéo)
- 3 loại Heuristic: Manhattan, Euclidean (mặc định), Diagonal/Octile
- Weighted A* (tăng tốc độ tìm đường bằng heuristic_weight > 1.0)
- Obstacle Inflation Cost — phạt cell gần vật cản, tạo đường đi an toàn
- Path Simplification — loại bỏ waypoint thẳng hàng (Collinearity Check)
- Replan định kỳ — tự động tìm lại đường khi map SLAM thay đổi
- Tích hợp Frontier-Based Exploration + DWA Local Planner
- Tương thích ROS2

## Yêu cầu

- ROS2 Humble 22.04
- Gazebo
- Rviz
- SLAM
- Frontier Based


## Kiến trúc

```
OccupancyGrid ──► AStarMap ──► AStarPlanner ──► AStarGlobalPlanner ──► Path
    (SLAM)     (Wrapper Grid)   (A* Algorithm)    (Coordinator)      (Waypoints)

                                                      │
                                                      ▼
                                                PathSimplifier
                                             (Collinearity Check)
```

### Vòng lặp Global Planning

```
Mỗi chu kỳ 500ms (2 Hz):

1. updateMap(grid)     — Cập nhật OccupancyGrid mới nhất từ SLAM

2. setGoal(gx, gy)    — Đặt mục tiêu mới (từ Frontier hoặc người dùng)

3. compute(x, y, θ)   — Cơ chế 2 lớp:
   [a] Replan khi cần:
       - Mới nhận goal (has_pending_replan_ = true)
       - Đến waypoint cuối nhưng chưa đến goal
       - Định kỳ mỗi replan_interval bước (20 bước × 200ms = 4s)
   [b] Bám theo đường:
       - Kiểm tra robot đến waypoint hiện tại chưa → advance
       - Chỉ cập nhật state nội bộ (current_wp_idx_)
```

## Cấu trúc thư mục

```
include/
├── astar_config.h             # Tham số cấu hình (heuristic, obstacle, navigation)
├── astar_map.h                # Wrapper OccupancyGrid + Inflation Cost
├── astar_node.h               # Cấu trúc node A* (x, y, f, g, h, parent)
├── astar_path.h               # Đường đi + Path Simplifier (Collinearity Check)
├── astar_planner.h            # Thuật toán A* trên lưới 8-connectivity
└── astar_global_planner.h     # Coordinator chính (replan, waypoint tracking)

cpp/
├── astar_map.cpp
├── astar_path.cpp
├── astar_planner.cpp
└── astar_global_planner.cpp
```

## API

### Thuật toán (Algorithm)

#### A* Heuristic

A* sử dụng hàm chi phí: **f(n) = g(n) + h(n)**

| Thành phần | Mô tả | Công thức |
|-----------|-------|-----------|
| `g(n)` | Chi phí thực tế từ Start đến n | g(next) = g(cur) + move_cost + obstacle_penalty |
| `h(n)` | Heuristic ước lượng từ n đến Goal | (xem bảng dưới) |
| `f(n)` | Tổng chi phí ước tính | f(n) = g(n) + h(n) |

#### Chi phí di chuyển (Move Cost) — 8-connectivity

| Hướng | Delta (dx, dy) | Cost |
|-------|---------------|------|
| Ngang / Dọc | (1,0), (0,1), (-1,0), (0,-1) | `1.0` |
| Chéo | (1,1), (1,-1), (-1,1), (-1,-1) | `√2 ≈ 1.4142` |

#### Heuristic (Hàm ước lượng)

Đặt: `dx = |goal_x - x|`, `dy = |goal_y - y|`

| Loại | Công thức | Đặc điểm |
|------|-----------|---------|
| **MANHATTAN** | `h = dx + dy` | 4-connectivity, admissible |
| **EUCLIDEAN** (mặc định) | `h = √(dx² + dy²)` | 8-connectivity, admissible |
| **DIAGONAL / OCTILE** | `h = max(dx,dy) + (√2-1)·min(dx,dy)` | 8-connectivity, chính xác nhất |

#### Obstacle Inflation Cost

Với mỗi cell (x, y), tính khoảng cách `d` đến vật cản gần nhất trong bán kính `safety_margin` cells:

```
cost = obstacle_penalty × (1 - d / safety_margin)   nếu d < safety_margin
     = 0.0                                            nếu không có vật cản gần
```

Giúp A* tìm đường đi an toàn, cách xa tường và vật cản.

#### Path Simplification (Collinearity Check)

Loại bỏ các waypoint trung gian trên đoạn thẳng:

```
Với 3 điểm liên tiếp A(x1,y1), B(x2,y2), C(x3,y3):

cross = (B.x - A.x)*(C.y - A.y) - (B.y - A.y)*(C.x - A.x)

Nếu |cross| / |AC| < simplify_tolerance → A, B, C thẳng hàng → loại B
```

Kết quả: Giảm từ N waypoint xuống K waypoint (K ≤ N), chỉ giữ lại các điểm góc cua.

### Topic

**Lưu ý:** Thư viện A* là một thư viện thuật toán (computational library), không phải một ROS node độc lập. Nó được tích hợp trong node `AutonomousExploration` và sử dụng các topic sau thông qua node đó:

| Topic | Kiểu dữ liệu | Chiều | Tần suất | Mô tả |
|-------|-------------|-------|---------|-------|
| `/map` | `nav_msgs::msg::OccupancyGrid` | Subscribe | 2 Hz | Bản đồ occupancy grid từ SLAM, dùng để xây dựng AStarMap và tìm đường. |
| `/autonomous_exploration/global_path` | `nav_msgs::msg::Path` | Publish | 2 Hz | Đường đi toàn cục dạng PoseArray các waypoint từ A* global planner (frame_id: `map`). |
| `/autonomous_exploration/astar_waypoints` | `visualization_msgs::msg::MarkerArray` | Publish | 2 Hz | Trực quan hóa waypoint trên Rviz với 4 màu phân biệt: xanh lá (hiện tại), đỏ (goal), xám (đã đi), xanh dương (chưa đi). Kèm TEXT_VIEW_FACING label. |

### Subscribe 

| Topic | Kiểu dữ liệu | QoS | Callback | Mô tả |
|-------|-------------|-----|----------|-------|
| `/map` | `nav_msgs::msg::OccupancyGrid` | `transient_local` QoS (1) | `mapCallback()` → `updateMap(grid)` | Nhận occupancy grid từ SLAM. Cập nhật AStarMap nội bộ để A* tìm đường trên bản đồ mới nhất. |

Ngoài ra, `AStarGlobalPlanner` còn sử dụng **TF2** thông qua node chủ:

| TF Frame | Chiều | Mô tả |
|----------|-------|-------|
| `map` → `base_link` | Nhận (Lookup) | Lấy pose robot để làm điểm start cho A*, kiểm tra waypoint và goal. |

### Publish 

| Topic | Kiểu dữ liệu | QoS | Mô tả |
|-------|-------------|-----|-------|
| `/autonomous_exploration/global_path` | `nav_msgs::msg::Path` | `transient_local` QoS (5) | Đường đi từ A* dưới dạng nav_msgs/Path. Mỗi waypoint là PoseStamped với z=0.02. Được publish bởi `publishGlobalPath()`. |
| `/autonomous_exploration/astar_waypoints` | `visualization_msgs::msg::MarkerArray` | `transient_local` QoS (5) | MarkerArray gồm các SPHERE đại diện cho waypoint với màu sắc: xanh lá (waypoint hiện tại, scale 0.20), đỏ (goal, scale 0.22), xám (đã đi qua, scale 0.08, alpha 0.4), xanh dương (chưa đi, scale 0.12). Kèm nhãn TEXT_VIEW_FACING cho waypoint quan trọng. |

### Sơ đồ luồng dữ liệu (Data Flow Diagram)

```
                        ┌──────────────────────────┐
                        │     SLAM Robot Node       │
                        │  (slam_robot)             │
                        └────────────┬─────────────┘
                                     │ /map (OccupancyGrid) 2 Hz
                                     ▼
┌───────────────────────────────────────────────────────────────────┐
│                  AutonomousExploration Node                       │
│                                                                   │
│  ┌──────────────────────────────────────────────────┐             │
│  │           globalPlannerTimerCallback (2 Hz)      │             │
│  │                                                  │             │
│  │  1. mapCallback() → global_planner_.updateMap()  │             │
│  │     ▲ Chuyển ROS OccupancyGrid → AStarMap        │             │
│  │                                                  │             │
│  │  2. Lấy goal từ FrontierExplorer:                │             │
│  │     frontier_explorer_.getGoalX/Y()              │             │
│  │     → global_planner_.setGoal(gx, gy)            │             │
│  │     ▲ Trigger replan ngay lập tức                │             │
│  │                                                  │             │
│  │  3. Lấy pose robot từ TF (map→base_link)         │             │
│  │                                                  │             │
│  │  4. global_planner_.compute(x, y, θ, step):      │             │
│  │     [a] Kiểm tra cần replan?                     │             │
│  │         - Goal mới? → yes                        │             │
│  │         - Hết waypoint? → yes                    │             │
│  │         - Định kỳ 20 bước (4s)? → yes            │             │
│  │     [b] Nếu cần replan:                          │             │
│  │         runAstar(robot_x, robot_y)               │             │
│  │         ├─ worldToGrid: robot, goal → grid       │             │
│  │         ├─ AStarPlanner::plan()                  │             │
│  │         │  ├─ Open Set (min-heap theo f)         │             │
│  │         │  ├─ Closed Set (visited)               │             │
│  │         │  ├─ N_8 neighbors                      │             │
│  │         │  ├─ g = g + move_cost + obstacle_cost  │             │
│  │         │  └─ Trace-back path → grid cells       │             │
│  │         ├─ gridToWorld: grid cells → world (m)   │             │
│  │         └─ PathSimplifier::simplify()            │             │
│  │            (Collinearity Check: loại B thẳng     │             │
│  │             hàng giữa A và C)                    │             │
│  │                                                  │             │
│  │     [c] Bám waypoint:                            │             │
│  │         - Nếu đến waypoint hiện tại → advance    │             │
│  │         - Nếu đến goal cuối → signal Frontier    │             │
│  │                                                  │             │
│  │  5. publishGlobalPath() + publishAStarWaypoints()│             │
│  └──────────────────┬───────────────────────────────┘             │
│                     │ path (waypoints)                            │
│                     ▼                                             │
│  ┌──────────────────────────────────────────┐                     │
│  │     DWA Local Planner (5 Hz)              │                    │
│  │  • Nhận path từ A*                        │                    │
│  │  • Tính cmd_vel để bám waypoint           │                    │
│  │  • Tránh vật cản động bằng LiDAR          │                    │
│  └──────────────────┬───────────────────────┘                     │
│                     │ /cmd_vel (Twist)                            │
│                     ▼                                             │
│              ┌─────────────┐                                      │
│              │   Robot     │                                      │
│              └─────────────┘                                      │
└───────────────────────────────────────────────────────────────────┘

Visualization:
┌──────────────────────────────────────────────────────────────┐
│  /autonomous_exploration/global_path     ────► 2 Hz (Rviz)   │
│  /autonomous_exploration/astar_waypoints ────► 2 Hz (Rviz)   │
└──────────────────────────────────────────────────────────────┘
```

### Chiến lược Replan

| Điều kiện | Mô tả |
|-----------|-------|
| **Goal mới** | `setGoal()` được gọi → `has_pending_replan_ = true` → replan ngay |
| **Hết waypoint** | Đến waypoint cuối nhưng chưa đến goal → replan |
| **Định kỳ** | Mỗi `replan_interval` bước (20 × 200ms = 4 giây) → replan do map SLAM thay đổi |
| **A* thất bại** | Giữ path cũ, log warning, không làm robot dừng đột ngột |

## Tham số

| Tham số | Giá trị | Mô tả |
|---------|----------|-------|
| `heuristic` | `EUCLIDEAN` | Loại heuristic: MANHATTAN / EUCLIDEAN / DIAGONAL |
| `heuristic_weight` | 1.0 | Trọng số heuristic (1.0 = A* tối ưu, >1.0 = Weighted A* nhanh hơn) |
| `obstacle_penalty` | 5.0 | Chi phí phạt khi đi qua cell gần vật cản |
| `safety_margin` | 8 cells (0.40m) | Bán kính vùng đệm an toàn xung quanh vật cản |
| `waypoint_tolerance` | 0.20 m | Khoảng cách để chuyển sang waypoint tiếp theo |
| `goal_tolerance` | 0.30 m | Sai số cho phép khi đến goal |
| `max_iterations` | 100,000 | Giới hạn số vòng lặp A* (tránh treo) |
| `replan_interval` | 20 bước (4s) | Khoảng cách giữa 2 lần replan định kỳ |
| `simplify_tolerance` | 0.01 m | Ngưỡng bỏ điểm thẳng hàng (Collinearity Check) |
| `max_waypoint_spacing` | 0.35 m | Khoảng cách tối đa giữa các waypoint (chèn thêm nếu cần) |

## Tài liệu tham khảo

- Hart, P. E. et al. (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*. IEEE Trans. Syst. Sci. Cybern.
- Hart, P. E. et al. (1972). *A Correction to "A Formal Basis for the Heuristic Determination of Minimum Cost Paths"*. SIGART Newsletter.
- Russell, S. & Norvig, P. (2021). *Artificial Intelligence: A Modern Approach* (4th ed.). Pearson.
