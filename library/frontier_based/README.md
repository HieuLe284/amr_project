# Frontier-Based Exploration

Thư viện Frontier-Based Exploration 2D nhẹ, viết bằng C++17 cho robot ROS2, dựa trên thuật toán Yamauchi (1997).

## Tính năng

- Phát hiện Frontier (Wave-Front BFS) — tìm ranh giới giữa vùng đã biết và chưa biết
- Gom cụm Frontier Region — gom các frontier cell liền kề (8-connectivity) thành cluster
- Chọn Frontier tối ưu — cost function cân bằng giữa khoảng cách & kích thước
- Safe Goal Projection — ép centroid về free cell gần nhất nếu nằm trên vật cản/unknown
- Noise Filtering — lọc frontier cell sát tường (safe_radius = 3 cells ≈ 15cm)
- Tích hợp SLAM + A* Global Planner + DWA Local Planner
- Phát hiện và thoát kẹt (Stuck Detection & Recovery) 3-phase
- Tương thích ROS2

## Yêu cầu

- ROS2 Humble 22.04
- Gazebo
- Rviz
- Thuật toán SLAM 

## Kiến trúc

```
OccupancyGrid ──► FrontierDetector ──► FrontierSelector ──► FrontierGoal
    (SLAM)      (Wave-Front BFS)     (Cost Function)      (x, y)

                     │
                     ▼
              A* Global Planner ──► DWA Local Planner ──► /cmd_vel
              (Euclidean Heuristic)  (Velocity Sampling)   (Twist)
```

## Cấu trúc thư mục

```
include/
├── frontier_config.h              # Tham số cấu hình
├── frontier_map.h                 # Wrapper OccupancyGrid
├── frontier_detector.h            # Phát hiện frontier (Wave-Front BFS)
├── frontier_selector.h            # Chọn frontier tốt nhất
└── frontier_exploration.h         # Điều phối chính

cpp/
├── frontier_config.cpp
├── frontier_map.cpp
├── frontier_detector.cpp
├── frontier_selector.cpp
└── frontier_exploration.cpp
```

## API
### Topic 

| Topic | Kiểu dữ liệu | Chiều | Tần suất | Mô tả |
|-------|-------------|-------|---------|-------|
| `/map` | `nav_msgs::msg::OccupancyGrid` | Subscribe | 2 Hz | Bản đồ occupancy grid từ SLAM, dùng để phát hiện frontier và tính frontier goal. |
| `/autonomous_exploration/frontier_markers` | `visualization_msgs::msg::MarkerArray` | Publish | 2 Hz | Gồm SPHERE (centroid), TEXT_VIEW_FACING (label), ARROW (robot→goal). |

### Subscribe

| Topic | Kiểu dữ liệu | QoS | Callback | Mô tả |
|-------|-------------|-----|----------|-------|
| `/map` | `nav_msgs::msg::OccupancyGrid` | `transient_local` QoS (1) | `mapCallback()` | Nhận occupancy grid từ SLAM. Lưu vào bộ đệm (cache) cho `frontier_explorer_.update(grid)` — cập nhật map nội bộ cho Wave-Front BFS. |

Ngoài ra, thư viện frontier còn yêu cầu **TF2** để nhận pose robot:

| TF Frame | Chiều | Mô tả |
|----------|-------|-------|
| `map` → `base_link` | Nhận (Lookup) | Lấy pose hiện tại của robot trong hệ tọa độ bản đồ. Được lookup tại `frontierTimerCallback()` (2Hz) và truyền vào `frontier_explorer_.compute(x, y, theta)`. |

### Publish

| Topic | Kiểu dữ liệu | QoS | Mô tả |
|-------|-------------|-----|-------|
| `/autonomous_exploration/frontier_markers` | `visualization_msgs::msg::MarkerArray` | `transient_local` QoS (10) | Visualization markers gồm 3 loại: SPHERE (centroid các frontier region — vàng cho best, xanh dương cho các region khác), TEXT_VIEW_FACING (label "F0(5c)" kèm kích thước region), ARROW (chỉ hướng robot → goal màu xanh lá). |

## Thuật toán

### 1. Phát hiện Frontier — Wave-Front Propagation (BFS)

Thuật toán phát hiện frontier dựa trên **Wave-Front Propagation (WFD)** — một biến thể của BFS trên không gian FREE, bắt đầu từ vị trí robot.

#### 1.1. Định nghĩa Frontier Cell

Một ô lưới `c = (x, y)` là **Frontier Cell** khi và chỉ khi:

```
P(x, y) == 0          (FREE — không có vật cản)
Và ∃ n ∈ N₈(x,y): P(n) == -1    (có ít nhất 1 neighbor UNKNOWN trong 8-connectivity)
```

Trong đó:
- `P(x, y)` — giá trị occupancy tại ô (x, y) theo ROS convention: `0 = FREE`, `-1 = UNKNOWN`, `1-100 = OCCUPIED`
- `N₈(x, y)` — tập 8 ô lân cận (x±1, y±1)

```
    Minh họa: Frontier Cell
    ┌───┬───┬───┐
    │ ? │ ? │ ? │    ? = UNKNOWN
    ├───┼───┼───┤    . = FREE
    │ ? │ . │ . │    F = Frontier Cell (FREE + có neighbor UNKNOWN)
    ├───┼───┼───┤
    │ ? │ F │ . │
    └───┴───┴───┘
```

**Kiểm tra nhanh:** Chỉ cần 1 neighbor UNKNOWN là đủ → `break` ngay, giảm tính toán.

#### 1.2. Noise Filtering — Safe Radius

Lọc nhiễu: Frontier không được phép nằm sát vật cản. Điều này cắt đứt "perimeter ring" chạy dọc tường do nhiễu tia laser hoặc lỗi tia không đến đích.

```
Điều kiện: ∀(dx, dy) ∈ [-3, 3]²: ¬isOccupied(x + dx, y + dy)
```

Nghĩa là: trong bán kính **3 cells (~15cm với resolution = 0.05m)** xung quanh frontier cell, không được có cell OCCUPIED nào. Nếu có, cell đó bị loại khỏi frontier.

```
    Ví dụ: Frontier bị loại do sát tường
    ┌───┬───┬───┬───┬───┐
    │ ■ │ ■ │ ■ │ ? │ ? │    ■ = OCCUPIED (tường)
    ├───┼───┼───┼───┼───┤    ? = UNKNOWN
    │ ■ │ F │ . │ . │ ? │    F = frontier cell bị loại
    ├───┼───┼───┼───┼───┤        (vì có ■  trong safe_radius=3)
    │ ■ │ . │ . │ . │ ? │
    └───┴───┴───┴───┴───┘
```

#### 1.3. Wave-Front BFS trên FREE Space

```
Input:  FrontierMap M, robot pose (x_R, y_R)
Output: Danh sách frontier cells reachable

Bước 1 - Khởi tạo:
  Chuyển (x_R, y_R) → (gx, gy) bằng worldToGrid()
  Nếu cell robot không FREE → tìm cell FREE gần nhất
    trong bán kính 10 cells bằng argmin(dx² + dy²)

Bước 2 - BFS (4-connectivity):
  Queue ← {(gx, gy)}
  Với mỗi cell (cx, cy) được dequeue:
    Nếu isFrontierCell(cx, cy) → thêm vào frontier_cells_found
    Với mỗi neighbor (nx, ny) trong 4-connectivity:
      Nếu neighbor FREE và chưa visited → enqueue

Bước 3 - Kết thúc BFS:
  Trả về danh sách frontier_cells_found

Độ phức tạp: O(W × H) với W×H là kích thước occupancy grid
```

**Tại sao dùng 4-connectivity cho BFS FREE space?**
- 4-connectivity nhanh hơn 8-connectivity (4 neighbors vs 8 neighbors)
- Đủ để duyệt toàn bộ không gian FREE reachable
- Frontier detection vẫn dùng 8-connectivity (cho cả isFrontierCell và clustering)

#### 1.4. Xử lý Robot nằm trên cell không FREE

Nếu robot nằm trên cell OCCUPIED hoặc UNKNOWN (do nhiễu hoặc inflation_radius của costmap), thuật toán sẽ tìm cell FREE gần nhất bằng Euclidean distance:

```
best = argmin(dx² + dy²) với (dx, dy) ∈ [-10, 10]², isFree(nx, ny) = true
```

### 2. Gom cụm Frontier Region — BFS 8-connectivity

Từ danh sách frontier cells, thuật toán gom các cell liền kề thành từng **FrontierRegion** bằng BFS 8-connectivity:

```
Mỗi frontier cell chưa được cluster:
  1. Tạo FrontierRegion mới
  2. BFS trên frontier cells (8-connectivity):
     - Đánh dấu visited
     - Thêm cell vào region
     - Cộng dồn tọa độ world để tính centroid
  3. Tính centroid của region
  4. Nếu region.size() >= min_frontier_size (mặc định: 5):
       → Giữ lại
     Ngược lại:
       → Loại bỏ (nhiễu)
```

#### 2.1. Công thức tính Centroid

Tọa độ trọng tâm (centroid) của một FrontierRegion:

```
X_G  = (1 / S) × Σⱼ(xⱼ)
Y_G  = (1 / S) × Σⱼ(yⱼ)
```

Trong đó:
- `S = |R|` — số frontier cells trong region
- `xⱼ, yⱼ` — tọa độ **world** (m) của từng frontier cell (gridToWorld)
- `X_G, Y_G` — centroid của region trong world frame

Mục đích: Robot navigation tới centroid thay vì đi tới từng cell → giảm dao động, path noise, local oscillation.

### 3. Chọn Frontier tối ưu — Cost Function

**Bài toán Multi-Objective Optimization:** Robot cần chọn frontier Rᵢ sao cho:
- **Tối thiểu hóa** khoảng cách di chuyển → `min(dist(robot, centroidᵢ))`
- **Tối đa hóa** kích thước frontier → `max(sizeᵢ)`

#### 3.1. Distance Normalization

```
Dᵢ = √((X_Gᵢ - X_R)² + (Y_Gᵢ - Y_R)²)

Dᵢ_norm = Dᵢ / maxⱼ(Dⱼ)    → [0, 1]
```

Trong đó:
- `Dᵢ` — khoảng cách Euclidean từ robot đến centroid của region thứ i
- `Dᵢ_norm` — khoảng cách đã chuẩn hóa

#### 3.2. Size Normalization

```
Sᵢ_norm = Sᵢ / maxⱼ(Sⱼ)    → [0, 1]
```

Trong đó:
- `Sᵢ` — kích thước (số cells) của frontier region thứ i
- `Sᵢ_norm` — kích thước đã chuẩn hóa

#### 3.3. Cost Function

```
Cᵢ = W_dist × Dᵢ_norm + W_size × (1 - Sᵢ_norm)
```

Trong đó:
- `W_dist = 0.7` — trọng số khoảng cách (ưu tiên frontier gần)
- `W_size = 0.3` — trọng số kích thước (ưu tiên frontier lớn)
- Cost thấp = frontier **GẦN + LỚN** → được ưu tiên

**Giải thích:**
- Frontier gần → `Dᵢ` nhỏ → `Dᵢ_norm` nhỏ → Cost nhỏ ✅
- Frontier lớn → `Sᵢ` lớn → `(1 - Sᵢ_norm)` nhỏ → Cost nhỏ ✅

#### 3.4. Ràng buộc lọc (Spatial Band-Pass Filter)

Trước khi tính cost, frontier bị loại nếu:

```
Dᵢ < 0.1m            → quá gần (nhiễu, lower-bound cutoff)
Dᵢ > max_goal_dist   → quá xa (mặc định: 50.0m)
```

Công thức lựa chọn cuối cùng:

```
R* = argmin Rᵢ (Cᵢ)
```

Nếu cost function không tìm được best (tất cả candidates có cost bằng nhau), fallback về F0 (frontier đầu tiên).

### 4. Safe Goal Projection

Vì centroid của một frontier region hình vòng cung (C-shaped) có thể rơi vào vùng OCCUPIED hoặc UNKNOWN, thuật toán "project" centroid về cell FREE gần nhất:

```
Input:  centroid (X_G, Y_G) từ FrontierRegion
Output: final_goal (x, y) đảm bảo nằm trên FREE space

1. Chuyển centroid → grid coordinates (gx, gy)
2. Nếu isFree(gx, gy) == true:
     → Giữ nguyên centroid
3. Ngược lại:
     → Quét ma trận 41×41 cells (bán kính 20 cells ≈ 1m)
     → Tìm cell FREE gần centroid nhất: argmin(dx² + dy²)
     → Chuyển cell đó về world coordinates: gridToWorld(best_gx, best_gy)
```

```
    Minh họa: C-shaped frontier
    ┌─────────────────────────┐
    │   ■ ■ ■ ■ ■ ■ ■ ■ ■     │    ■ = OCCUPIED (tường)
    │   ■ . . . . . . . ■     │    . = FREE (frontier cells)
    │   ■ . . . . . . . ■     │    ? = UNKNOWN
    │   ■ . . ★ . . . . ■    │    ★ = Centroid (rơi vào OCCUPIED)
    │   ■ . . F F F . ■ ? ?   │    F = Frontier cell
    │   ■ . . F F F . ■ ? ?   │    ● = Final goal sau projection
    │   ■ ■ ■ ■ ■ ■ ■ ■ ■     │
    └─────────────────────────┘
              ★ → ● (project về FREE gần nhất)
```

### 5. Vòng lặp thám hiểm (Exploration Loop)

```
Mỗi chu kỳ frontierTimerCallback (2 Hz):

1. update(grid) — Cập nhật OccupancyGrid mới nhất từ SLAM
   └→ map_.update(grid): copy data + metadata

2. compute(x, y, θ) — Tìm frontier goal mới nếu cần:
   a. Nếu chưa có goal (has_goal_ == false):
        → detect(): Wave-Front BFS → clustering → lọc
        → select(): cost function → best frontier
        → Nếu không có frontier hợp lệ → done_ = true
        → Project centroid về FREE gần nhất
        → Set goal_x_, goal_y_
   
   b. Nếu đang có goal:
        → Nếu A* báo goal_reached (signalGoalReached()):
            • Reset flag, tìm frontier mới
        → Nếu timeout > TIMEOUT (30s):
            • Abandon goal hiện tại, tìm frontier mới
        → Ngược lại: giữ nguyên goal, chờ A* hoàn thành

3. Nếu last_regions_ empty → done_ = true (exploration hoàn thành)

4. Frontier chỉ cập nhật state nội bộ (goal_x_, goal_y_).
   A* và DWA xử lý phần navigation còn lại.
```

## Sơ đồ luồng dữ liệu (Data Flow Diagram)

```
                ┌────────────────────┐
                │  SLAM Robot Node   │
                │  (slam_robot)      │
                └─────────┬──────────┘
                          │ /map (OccupancyGrid) 2 Hz
                          ▼
┌─────────────────────────────────────────────────────────┐
│               AutonomousExploration Node                │
│                                                         │
│  ┌─────────────────┐    ┌──────────────────┐            │
│  │  mapCallback()  │    │  scanCallback()  │            │
│  │  • Lưu occ_grid │    │  • Lưu LiDAR     │            │
│  │  • Update cache │    │  • Cache for DWA │            │
│  └────────┬────────┘    └────────┬─────────┘            │
│           │                      │                      │
│           ▼                      ▼                      │
│  ┌──────────────────────────────────────────┐           │
│  │         frontierTimerCallback (2 Hz)     │           │
│  │                                          │           │        
│  │  1. Lấy pose robot từ TF (map→base)      │           │        
│  │  2. frontier_explorer_.update(grid)      │           │        
│  │  3. frontier_explorer_.compute(x,y,θ)    │           │        
│  │     ├─ FrontierDetector.detect()         │           │        
│  │     │  (Wave-Front BFS: tìm frontier     │           │        
│  │     │   cells + cluster thành regions)   │           │        
│  │     ├─ FrontierSelector.select()         │           │        
│  │     │  (cost = w_dist*d_norm + w_size*   │           │        
│  │     │          (1-size_norm))            │           │        
│  │     └─ Safe Goal Projection              │           │        
│  │        (project centroid → FREE cell)    │           │        
│  │  4. publishFrontierMarkers()             │           │        
│  └──────────────────┬───────────────────────┘           │        
│                     │ goal (x, y)                       │        
│                     ▼                                   │         
│  ┌──────────────────────────────────────────┐           │        
│  │              A* Algorithm                │           │              
│  └──────────────────────────────────────────┘           │        
└─────────────────────────────────────────────────────────┘

Visualization:
┌──────────────────────────────────────────────────────────────────┐
│  /autonomous_exploration/frontier_markers  ────► 2 Hz (Rviz)     │
└──────────────────────────────────────────────────────────────────┘
```

## Tham số

| Tham số | Giá trị | Mô tả |
|---------|----------|-------|
| `min_frontier_size` | 5 | Số cell tối thiểu của một frontier region hợp lệ (lọc nhiễu) |
| `max_goal_dist` | 50.0 m | Khoảng cách tối đa để xét frontier (bỏ qua vùng biên quá xa) |
| `w_dist` | 0.7 | Trọng số khoảng cách trong cost function (ưu tiên frontier gần) |
| `w_size` | 0.3 | Trọng số kích thước trong cost function (ưu tiên frontier lớn) |
| `goal_tolerance` | 0.35 m | Sai số khoảng cách để coi đã đến goal |
| `TIMEOUT` | 30 s | Timeout khi frontier ở 1 vị trí quá lâu |

## Tài liệu tham khảo

- Yamauchi, B. (1997). *A Frontier-Based Approach for Autonomous Exploration*. IEEE Int. Symp. on Computational Intelligence in Robotics and Automation (CIRA).
- Hart, P. E. et al. (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*. IEEE Trans. Syst. Sci. Cybern.
- Fox, D. et al. (1997). *The Dynamic Window Approach to Collision Avoidance*. IEEE Robot. Autom. Mag.
