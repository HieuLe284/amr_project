# Graph SLAM / Pose Graph Optimization — slam_avoid_obstacle

## Tổng Quan

Chuyển đổi toàn bộ node từ **DWA (Dynamic Window Approach)** đơn thuần sang kiến trúc **Graph SLAM + Pose Graph Optimization** kết hợp **tránh vật cản**.

File `lidar_avoid_obstacle.cpp` sẽ được thay thế bằng `slam_avoid_obstacle.cpp` với tên node `slam_avoid_obstacle`.

---

## Nền Tảng Toán Học

### 1. Mô Hình Robot (Pose State)

Mỗi pose của robot tại thời điểm `k` được biểu diễn:

```
x_k = [x_k, y_k, θ_k]^T  ∈ ℝ³
```

Trong đó: `x` là vị trí ngang, `y` là vị trí dọc, `θ` là hướng robot (yaw).

---

### 2. Motion Model — Odometry Edge (Between Consecutive Poses)

Robot di chuyển theo **Differential Drive**, mô hình chuyển động:

```
x_{k+1} = f(x_k, u_k) + w_k

Với:
  x_{k+1} = x_k + Δx·cos(θ_k) - Δy·sin(θ_k)
  y_{k+1} = y_k + Δx·sin(θ_k) + Δy·cos(θ_k)
  θ_{k+1} = θ_k + Δθ

Trong đó:
  Δx = v · Δt · cos(w · Δt / 2)  ≈ v · Δt   (khi w nhỏ)
  Δy = v · Δt · sin(w · Δt / 2)  ≈ 0         (khi w nhỏ)
  Δθ = w · Δt

  w_k ~ N(0, Σ_odom)   (nhiễu Gaussian)
```

**Information Matrix của Odometry Edge:**

```
Ω_odom = Σ_odom^{-1}

Σ_odom = diag(σ_x², σ_y², σ_θ²)
       = diag(0.01, 0.01, 0.005)   (đơn vị: m², m², rad²)
```

---

### 3. Observation Model — LiDAR Landmark Edge

Mỗi scan LiDAR tại pose `x_k`, landmark `l_j = [lx_j, ly_j]^T`:

```
z_{k,j} = h(x_k, l_j) + v_{k,j}

Với:
  h_r(x_k, l_j) = sqrt((lx_j - x_k)² + (ly_j - y_k)²)      (khoảng cách)
  h_φ(x_k, l_j) = atan2(ly_j - y_k, lx_j - x_k) - θ_k      (góc bearing)

  v_{k,j} ~ N(0, Σ_obs)
  Σ_obs = diag(σ_r², σ_φ²) = diag(0.04, 0.01)   (m², rad²)
```

---

### 4. Pose Graph — Factor Graph Formulation

Bài toán SLAM được biểu diễn dưới dạng **Maximum A Posteriori (MAP)**:

```
X* = argmin_X  Σ_{(i,j)∈E_odom} e_{ij}^T · Ω_{ij} · e_{ij}
              + Σ_{(k,j)∈E_obs}  e_{kj}^T · Ω_{kj} · e_{kj}
```

**Error Function:**

```
── Odometry edge error (giữa 2 pose liên tiếp):
   e_{ij}(X) = x_j ⊖ f(x_i, u_{ij})
             = [ x_j - (x_i + Δx·cosθ_i - Δy·sinθ_i)  ]
               [ y_j - (y_i + Δx·sinθ_i + Δy·cosθ_i)  ]
               [ norm_angle(θ_j - θ_i - Δθ)             ]

── Observation edge error (pose → landmark):
   e_{kj}(X) = z_{kj,measured} - h(x_k, l_j)
```

---

### 5. Linearization — Gauss-Newton / Levenberg-Marquardt

Linearize error function quanh điểm ước lượng hiện tại `X̂`:

```
e(X̂ + δX) ≈ e(X̂) + J · δX

J = ∂e/∂X   (Jacobian matrix)
```

**Linear System:**

```
H · δX = -b

Trong đó:
  H = Σ J_i^T · Ω_i · J_i   (Hessian approximation, ma trận thưa)
  b = Σ J_i^T · Ω_i · e_i   (gradient vector)
```

**Levenberg-Marquardt damping:**

```
(H + λ·I) · δX = -b

λ: damping factor (tăng khi không hội tụ, giảm khi hội tụ)
```

**Update rule:**

```
X ← X ⊕ δX    (pose composition)
```

---

### 6. Jacobian của Odometry Edge

```
J_i = ∂e_{ij}/∂x_i =
  [ -1   0   -(Δx·(-sinθ_i) - Δy·cosθ_i)  ]
  [  0  -1   -(Δx·cosθ_i   - Δy·(-sinθ_i)) ]
  [  0   0    -1                             ]

J_j = ∂e_{ij}/∂x_j = I₃ₓ₃
```

---

### 7. Loop Closure Detection

Phát hiện khi robot quay lại vị trí đã từng thăm:

```
Điều kiện loop closure:
  dist(x_k, x_i) < d_thresh = 0.5 m
  |k - i| > gap_thresh = 10   (không phải cạnh liên tiếp)
```

Khi phát hiện loop closure → thêm **constraint edge** giữa `x_k` và `x_i`, kích hoạt optimization.

---

### 8. Scan Matching (ICP simplified)

Để tính relative pose giữa 2 scan:

```
T* = argmin_T  Σ_i ||p_i - T·q_i||²

Trong đó:
  p_i: điểm trong scan hiện tại (local frame)
  q_i: điểm correspondences trong scan tham chiếu
  T = [R | t]: Rotation + Translation
```

---

### 9. Tránh Vật Cản Tích Hợp Với SLAM

Sau khi optimize pose graph, robot sử dụng **estimated pose** để điều hướng:

```
Velocity command:
  v_cmd = v_max · (1 - exp(-k_v · dist_free))   (proportional to free space)
  w_cmd = -k_w · θ_error                         (proportional to heading error)

Điều kiện an toàn:
  Nếu ∃ điểm Lidar trong vùng nguy hiểm [r < r_safe]:
    v_cmd = 0, w_cmd = turn_speed   (quay tránh)
```

---

## Proposed Changes

### Component: SLAM Core

#### [DELETE] [lidar_avoid_obstacle.cpp](file:///u:/AGV_Robot/src/lidar_avoid_obstacle.cpp)

Xoá file cũ (DWA only, không có SLAM).

#### [NEW] slam_avoid_obstacle.cpp — `u:\AGV_Robot\src\slam_avoid_obstacle.cpp`

File C++ mới với các thành phần:

| Class/Struct        | Chức năng                                                   |
| ------------------- | ----------------------------------------------------------- |
| `Pose2D`            | Biểu diễn pose `[x, y, θ]`                                  |
| `OdomEdge`          | Cạnh odometry giữa 2 node trong pose graph                  |
| `PoseGraph`         | Quản lý toàn bộ pose graph (nodes + edges)                  |
| `GraphSLAMSolver`   | Gauss-Newton optimization, loop closure                     |
| `Slam_Robot`        | ROS2 Node chính: subscribe scan, publish cmd_vel, chạy SLAM |

**Logic chính:**

1. Mỗi khoảng thời gian `Δt`, tạo pose mới từ motion model → thêm vào graph
2. Subscribe LiDAR scan → extract obstacle points
3. Check loop closure → nếu có → thêm loop edge → chạy Graph Optimization
4. Tính cmd_vel dựa trên SLAM-estimated pose + scan data
5. Publish `/cmd_vel`

---

### Component: Build System

#### [MODIFY] [CMakeLists.txt](file:///u:/AGV_Robot/CMakeLists.txt)

- Xoá entry `lidar_avoid_obstacle`
- Thêm entry `slam_avoid_obstacle` trỏ vào `src/slam_avoid_obstacle.cpp`
- Thêm dependency: `tf2_geometry_msgs` (nếu cần)

---

## Open Questions

> [!IMPORTANT]
> **Eigenvalue Solver**: Để giải hệ `H·δX = -b`, tôi sẽ implement **Conjugate Gradient** đơn giản thuần C++ (không dùng thư viện ngoài như Eigen/g2o) để đảm bảo tương thích build hệ thống hiện tại. Bạn có muốn dùng Eigen3 không? (cần thêm `find_package(Eigen3)` vào CMakeLists).

> [!NOTE]
> **Loop Closure**: Phiên bản này implement loop closure dựa trên **Euclidean distance** giữa các pose. Trong production thực tế nên dùng scan matching (ICP), nhưng để tránh phức tạp dependencies, tôi sẽ dùng distance-based detection.

## Verification Plan

### Automated Build

```bash
cd /path/to/ros2_ws
colcon build --packages-select agv_robot
```

### Manual Verification

- Chạy `ros2 run agv_robot slam_avoid_obstacle`
- Xem log output: pose graph nodes, loop closures, optimization iterations
- Kiểm tra topic `/cmd_vel` có được publish không
