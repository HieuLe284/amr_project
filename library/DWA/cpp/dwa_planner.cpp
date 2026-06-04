#include "library/DWA/include/dwa_planner.h"

#include <cmath>
#include <iostream>

DWAPlanner::DWAPlanner(const DWAConfig& config) : config_(config) {}

// ================================================================
// Chuẩn hóa góc về [-π, π]
// ================================================================
static double normalizeAngle(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// ================================================================
// findLookaheadGoal — Tìm waypoint lookahead từ path và tính goal_angle nội bộ
// ================================================================
bool DWAPlanner::findLookaheadGoal(
    double robot_x, double robot_y, double robot_theta,
    const std::vector<std::pair<double, double>>& path,
    double& goal_angle) const
{
  if (path.empty()) return false;

  // Lookahead distance: 0.6m (~3 lần robot radius)
  const double LOOKAHEAD_DIST = 0.6;

  // Tìm waypoint đầu tiên cách robot >= lookahead_dist
  size_t lookahead_idx = 0;
  double min_dist = 1e9;
  for (size_t i = 0; i < path.size(); ++i) {
    double dx = path[i].first  - robot_x;
    double dy = path[i].second - robot_y;
    double d  = std::sqrt(dx*dx + dy*dy);
    if (d < min_dist) min_dist = d;
    if (d >= LOOKAHEAD_DIST) {
      lookahead_idx = i;
      break;
    }
  }

  // Nếu không tìm thấy waypoint nào đủ xa, lấy waypoint cuối cùng
  if (lookahead_idx == 0 && path.size() > 1) {
    lookahead_idx = path.size() - 1;
  }

  // Tính goal_angle từ robot đến waypoint lookahead, trong robot frame
  const auto& wp = path[lookahead_idx];
  double dx = wp.first  - robot_x;
  double dy = wp.second - robot_y;
  double world_angle = std::atan2(dy, dx);
  goal_angle = normalizeAngle(world_angle - robot_theta);

  return true;
}

// ================================================================
// computeVelocity — Nhận A* path
// ================================================================
std::pair<double, double> DWAPlanner::computeVelocity(
  const DWAState& state,
  const std::vector<float>& scan_ranges,
  double angle_min,
  double angle_increment,
  double yaw_offset,
  double robot_x,
  double robot_y,
  double robot_theta,
  const std::vector<std::pair<double, double>>& path)
{
  // ================================================================
  //  TÍNH GOAL_ANGLE NỘI BỘ từ full A* path
  // ================================================================
  double goal_angle = 0.0;
  bool has_goal = findLookaheadGoal(robot_x, robot_y, robot_theta, path, goal_angle);

  // Nếu path rỗng → dừng robot
  if (!has_goal) {
    return {0.0, 0.0};
  }

  // 1. Chuyển đổi Scan thành danh sách chướng ngại vật
  auto obstacles = scanToObstacles(scan_ranges, angle_min, angle_increment, yaw_offset);

  // ================================================================
  // 2. PREEMPTIVE ESCAPE — Kiểm tra nguy hiểm TRƯỚC scoring
  // Nếu có chướng ngại vật trong vùng nguy hiểm phía trước,
  // robot lập tức xoay thoát mà không chờ qua vòng sampling.
  //
  // Mục tiêu: "Khi cách vật cản X mét → xoay thoát ngay".
  // Khoảng cách kích hoạt escape: chỉ xét vật cản nằm trong
  // VỆT ĐƯỜNG của robot (|y| <= robot_radius).
  // ================================================================

  const double ESCAPE_TRIGGER_DIST = 0.35;
  // Quét vệt đường thẳng phía trước để tìm vật cản gần nhất
  double min_forward_dist = config_.sensor_max_range;

  for (const auto& obs : obstacles) {
    // Chỉ xét vật cản nằm phía trước
    if (obs.first <= 0.0) continue;

    // Chỉ xét vật cản nằm trong VỆT ĐƯỜNG của robot (theo chiều ngang)
    // → loại bỏ tường bên cạnh không nằm trên đường đi thẳng
    if (std::abs(obs.second) > config_.robot_radius) continue;

    // Dùng x (khoảng cách thẳng đứng về phía trước) làm tiêu chí trigger
    if (obs.first < min_forward_dist) {
      min_forward_dist = obs.first;
    }
  }

  // ================================================================
  // 3. PREEMPTIVE ESCAPE — Kiểm tra nguy hiểm TRƯỚC scoring
  //
  // Nếu có chướng ngại vật trong vùng nguy hiểm phía trước,
  // robot lập tức xoay thoát mà không chờ qua vòng sampling.
  //
  // Mục tiêu: "Khi cách vật cản X mét → xoay thoát ngay".
  // ================================================================
  if (min_forward_dist < ESCAPE_TRIGGER_DIST) {
    // === ESCAPE MODE ===
    // So sánh clearance của quỹ đạo rẽ trái vs rẽ phải
    double left_dist  = scorer_.computeDistance(0.08, config_.w_max * 0.6, obstacles, config_);
    double right_dist = scorer_.computeDistance(0.08, -config_.w_max * 0.6, obstacles, config_);

    double escape_w;
    if (std::abs(left_dist - right_dist) > 0.10) {
      // Chênh lệch rõ: chọn hướng clearance lớn hơn
      escape_w = (left_dist >= right_dist) ? config_.w_max * 0.9 : -config_.w_max * 0.9;
    } else {
      // Tương đương: dùng goal_angle để thoát về phía goal
      escape_w = (goal_angle > 0) ? config_.w_max * 0.9 : -config_.w_max * 0.9;
    }

    std::cout << "[DWA] PREEMPTIVE ESCAPE! forward_dist=" << min_forward_dist
              << "m left=" << left_dist << " right=" << right_dist
              << " escape_w=" << escape_w << std::endl;

    return {0.0, escape_w}; // v=0: dừng tiến, chỉ xoay thoát ngay
  }

  // ================================================================
  // 3. Tính Dynamic Window (Vd giao Vs)  — chế độ bình thường
  // ================================================================
  window_.compute(state, config_);

  // 4. Grid Sampling trong Dynamic Window
  const double dv = (window_.v_high - window_.v_low) / std::max(1, config_.v_samples - 1);
  const double dw = (window_.w_high - window_.w_low) / std::max(1, config_.w_samples - 1);

  double best_v   = 0.0;
  double best_w   = 0.0;
  double max_score = -1e9;

  for (int i = 0; i < config_.v_samples; ++i) {
    // duyệt trục tung
    double v_test = window_.v_low + i * dv;
    for (int j = 0; j < config_.w_samples; ++j) {
      // duyệt trục hoành
      double w_test = window_.w_low + j * dw;

      // Tính khoảng cách đến vật cản trên quỹ đạo cong (hoặc thẳng)
      double dist = scorer_.computeDistance(v_test, w_test, obstacles, config_);

      // ================================================================
      // 5. Admissible Velocities (Va):
      // Chỉ kiểm tra v (tịnh tiến) theo braking distance.
      // w KHÔNG kiểm tra riêng vì dist của arc trajectory nhỏ khi
      // đi sát tường là ĐÚNG — không nên reject nó.
      // Quãng đường phanh thắng tính từ lúc nhấn đến khi dừng hẳn: 
      // v_final^2 - v^2 = 2 * a * s => 0 - v^2 = -2 * d(v_dot_b)/dt * d_brake
      // => v = sqrt(2 * d(v_dot_b)/dt * d_brake)
      // Do đó Admissible Velocity là: 
      // V_a = {(v,ω) | |v| <= 2 * dist(v,ω) * d(v_bracke)/dt}
      // ================================================================
      bool is_admissible = true;
      if (std::abs(v_test) > 0.01) {
        if (std::abs(v_test) > std::sqrt(2.0 * dist * config_.v_dot_b + 1e-4)) {
          is_admissible = false;
        }
      }

      if (!is_admissible) continue;

      // ================================================================
      // 6. Objective Function:
      // G(v,w) = alpha * heading + beta * clearance + gamma * velocity
      // ================================================================
      double score = scorer_.score(state, v_test, w_test, dist, goal_angle, config_);

      if (score > max_score) {
        max_score = score;
        best_v    = v_test;
        best_w    = w_test;
      }
    }
  }

  // ================================================================
  // 7. Fallback: không có trajectory hợp lệ
  // ================================================================
  if (max_score < 0.0 || (best_v < 0.01 && std::abs(best_w) < 0.01)) {
    // Đếm vật cản gần ở trái/phải
    int left_count  = 0;
    int right_count = 0;
    for (const auto& obs : obstacles) {
      double obs_dist = std::sqrt(obs.first * obs.first + obs.second * obs.second);
      if (obs_dist < config_.robot_radius * 4.0) {
        if (obs.second > 0) left_count++;
        else right_count++;
      }
    }

    // Lùi nhẹ để tạo khoảng trống + xoay thoát
    best_v = -0.05;
    if (std::abs(goal_angle) > 0.15) {
      best_w = (goal_angle > 0) ? config_.w_max * 0.8 : -config_.w_max * 0.8;
    } else {
      best_w = (left_count <= right_count) ? config_.w_max * 0.8 : -config_.w_max * 0.8;
    }

    std::cout << "[DWA] FALLBACK! goal_angle=" << goal_angle
              << " v=" << best_v << " w=" << best_w << std::endl;
  }

  return {best_v, best_w};
}

// ================================================================
// 8. Chuyển đổi Scan thành danh sách vật cản trong robot frame
// Phép chuyển hệ tọa độ cực (Polar to Cartesian Map Projection)
//  θ_i = θ_min + i * Δθ_increment + θ_offset
// Phép xoay chiếu góc lên tọa độ phẳng
//  x_i = r_i * cos(θ_i)
//  y_i = r_i * sin(θ_i)
// ================================================================
std::vector<std::pair<double, double>> DWAPlanner::scanToObstacles(
  const std::vector<float>& ranges,
  double angle_min,
  double angle_increment,
  double yaw_offset) const
{
  std::vector<std::pair<double, double>> obstacles;
  obstacles.reserve(ranges.size() / 3 + 1);

  // Chia đôi dữ liệu quét tia Lidar +=2 nhằm tiết kiệm 50% CPU tính toán
  //mà không bị ảnh hưởng tới kết quả của DWA
  for (size_t i = 0; i < ranges.size(); i += 2) {
    float r = ranges[i];

    // Bỏ qua tia lỗi hoặc chính thân xe (r < 0.12m)
    if (std::isinf(r) || std::isnan(r) || r < 0.12f || r > config_.sensor_max_range) {
      continue;
    }

    //Phép chuyển hệ tọa độ cực
    double angle = angle_min + i * angle_increment + yaw_offset;

    //Phép chuẩn hóa góc về khoảng [-π, π]
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;

    // Phép xoay chiếu góc lên tọa độ phẳng
    double x = r * std::cos(angle);
    double y = r * std::sin(angle);

    obstacles.emplace_back(x, y);
  }

  return obstacles;
}
