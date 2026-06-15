#include "include/autonomous_exploration.h"

using std::placeholders::_1;

// ════════════════════════════════════════════════════════════════════════════
//  Constructor
// ════════════════════════════════════════════════════════════════════════════
AutonomousExploration::AutonomousExploration() : Node("autonomous_exploration") {
    // ── Callback groups ───────────────────────────────────────────────────
    callback_group_frontier_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // frontier
    callback_group_global_planner_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // A* global planner
    callback_group_local_planner_  =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // DWA local planner

    // ── TF2 ──────────────────────────────────────────────────────────────
    tf_buffer_      = std::make_shared<tf2_ros::Buffer>(this->get_clock());      
    tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ── Publishers ────────────────────────────────────────────────────────
    pub_cmd_    = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); // moving

    pub_frontier_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/autonomous_exploration/frontier_markers", rclcpp::QoS(10).transient_local());    // Frontier

    pub_global_path_ = create_publisher<nav_msgs::msg::Path>(
        "/autonomous_exploration/global_path", rclcpp::QoS(5).transient_local());          // A* Global Path

    pub_astar_waypoints_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/autonomous_exploration/astar_waypoints", rclcpp::QoS(5).transient_local());      // A* Waypoint visualization

    // ── Subscribers ───────────────────────────────────────────────────────
    // Nhận map từ slam_robot
    sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local(),
        std::bind(&AutonomousExploration::mapCallback, this, _1));

    // Nhận LiDAR scan cho DWA
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/agv_scan", rclcpp::SensorDataQoS(),
        std::bind(&AutonomousExploration::scanCallback, this, _1));

    // ── Timers ────────────────────────────────────────────────────────────
    // 500ms = 2 Hz: frontier exploration timer
    frontier_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&AutonomousExploration::frontierTimerCallback, this),
        callback_group_frontier_);

    // 500ms = 2 Hz: A* global planner timer
    global_planner_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&AutonomousExploration::globalPlannerTimerCallback, this),
        callback_group_global_planner_);

    // 200ms = 5 Hz: DWA local planner timer
    local_planner_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&AutonomousExploration::localPlannerTimerCallback, this),
        callback_group_local_planner_);

    RCLCPP_INFO(get_logger(),
        "[AutonomousExploration] Frontier-Based Exploration initialized (Yamauchi, 1997).");
    RCLCPP_INFO(get_logger(),
        "[AutonomousExploration] Library: lib/frontier_based | Detector: Wave-Front BFS");
    RCLCPP_INFO(get_logger(),
        "[AutonomousExploration] A* Global Planner initialized (Hart et al., 1968).");
    RCLCPP_INFO(get_logger(),
        "[AutonomousExploration] Library: lib/A_star_algorithm | Heuristic: Euclidean");
    RCLCPP_INFO(get_logger(),
        "[AutonomousExploration] DWA Local Planner initialized (Fox, Burgard, Thrun, 1997).");
    RCLCPP_INFO(get_logger(),
        "[AutonomousExploration] Library: lib/DWA | Objective: alpha*heading + beta*clearance + gamma*vel");

    // Bật chế độ tự động thám hiểm
    exploration_mode_.store(true);
}

// ════════════════════════════════════════════════════════════════════════════
//  mapCallback — nhận OccupancyGrid từ slam_robot
// ════════════════════════════════════════════════════════════════════════════
void AutonomousExploration::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    cached_occ_grid_ = *msg;
    cached_grid_valid_ = true;
    if (!map_initialized_) {
        map_initialized_ = true;
        RCLCPP_INFO(get_logger(), "[AutonomousExploration] Received first map from slam_robot.");
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  scanCallback - lưu LiDAR scan cho DWA
// ════════════════════════════════════════════════════════════════════════════
void AutonomousExploration::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    cached_scan_ranges_.assign(scan->ranges.begin(), scan->ranges.end());
    cached_scan_angle_min_       = scan->angle_min;
    cached_scan_angle_increment_ = scan->angle_increment;
}

// ════════════════════════════════════════════════════════════════════════════
//  ----------------------------- FRONTIER BASED ------------------------------
// ════════════════════════════════════════════════════════════════════════════
//  Nguyên lý hoạt động:
//    1. Robot duy trì occupancy grid M (FREE / OCCUPIED / UNKNOWN)
//    2. Frontier cell = FREE cell có ít nhất 1 neighbor UNKNOWN
//    3. Frontier region = cluster các frontier cell liền kề (BFS)
//    4. Frontier goal = centroid của frontier region tốt nhất
//       (cost function: cân bằng giữa khoảng cách & kích thước)
//    5. Robot di chuyển đến frontier goal → lặp lại
//    6. Khi không còn frontier → exploration hoàn thành
// ════════════════════════════════════════════════════════════════════════════

// frontierTimerCallback — 2 Hz: chạy frontier exploration
void AutonomousExploration::frontierTimerCallback()
{
    if (!map_initialized_) return;         // Chưa có map → chưa làm gì
    if (!exploration_mode_.load()) return; // Chưa bật exploration

    // ── Lấy pose robot hiện tại (map → base_link) ────────────────────────
    double rx, ry, rtheta;
    try {
        auto tf = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time());
        rx = tf.transform.translation.x;
        ry = tf.transform.translation.y;
        tf2::Quaternion q(tf.transform.rotation.x,
                          tf.transform.rotation.y,
                          tf.transform.rotation.z,
                          tf.transform.rotation.w);
        rtheta = getYaw(q);
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[Frontier] Cannot get map→base TF: %s", ex.what());
        return;
    }

    // ── Cập nhật OccupancyGrid mới nhất vào frontier map ─────────────────
    if (!cached_grid_valid_) {
        return; // Nếu chưa có cached grid, bỏ qua frontier lần này
    }

    // ── Lấy dữ liệu từ SLAM ──────────────────────────────────────────────
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        frontier_explorer_.update(cached_occ_grid_);
    }

    // ── Gọi compute để tìm / cập nhật frontier goal ──────────────────────
    Pose2D cur(rx, ry, rtheta);
    slam_exploration(cur);

    // ── Publish frontier markers lên RViz ────────────────────────────────
    const FrontierRegion* best = nullptr;
    double bgx = cached_goal_x_;
    double bgy = cached_goal_y_;
    if (cached_has_goal_) {
        // Tìm best region từ cached_regions_ (region có centroid gần goal nhất)
        double min_dist = 1e9;
        for (const auto& r : cached_regions_) {
            double d = std::hypot(r.centroid_x - bgx, r.centroid_y - bgy);
            if (d < min_dist) {
                min_dist = d;
                best = &r;
            }
        }
    }
    publishFrontierMarkers(cached_regions_, best, cached_robot_x_, cached_robot_y_,
                           cached_goal_x_, cached_goal_y_);
}

// ---- Frontier Exploration ----
void AutonomousExploration::slam_exploration(Pose2D& cur) {
    if (!exploration_mode_.load()) return;

    // Gọi frontier_explorer_ để tìm frontier goal dựa trên occupancy grid + pose robot
    frontier_explorer_.compute(cur.x, cur.y, cur.theta);

    // Lưu kết quả vào cache để publish markers
    cached_regions_ = frontier_explorer_.getLastRegions();
    cached_robot_x_ = cur.x;
    cached_robot_y_ = cur.y;
    cached_goal_x_  = frontier_explorer_.getGoalX();
    cached_goal_y_  = frontier_explorer_.getGoalY();
    cached_has_goal_ = frontier_explorer_.hasGoal();

    // Log thông tin frontier
    if (cached_has_goal_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[Frontier] Goal: (%.2f, %.2f) | Regions: %zu | Mode: %s",
            cached_goal_x_, cached_goal_y_,
            cached_regions_.size(),
            exploration_mode_.load() ? "EXPLORING" : "IDLE");
    } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "[Frontier] No goal available. Regions: %zu", cached_regions_.size());
    }

    // Kiểm tra exploration đã hoàn thành chưa
    if (frontier_explorer_.isDone()) {
        RCLCPP_INFO(get_logger(), "[Frontier] === Exploration DONE ===");
        exploration_mode_.store(false);
    }
}

// ---- Frontier Markers ----
void AutonomousExploration::publishFrontierMarkers(
    const std::vector<FrontierRegion>& regions, const FrontierRegion* best,
    double robot_x, double robot_y, double goal_x, double goal_y)
{
  visualization_msgs::msg::MarkerArray arr;
  auto now = this->now();

  // Xóa tất cả markers cũ
  visualization_msgs::msg::Marker del_all;
  del_all.header.frame_id = "map"; del_all.header.stamp = now;
  del_all.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del_all);

  int id = 0;

  // Vẽ từng frontier region dưới dạng SPHERE + TEXT
  for (size_t i = 0; i < regions.size(); ++i) {
    const auto& r = regions[i];
    bool is_best = (best && std::abs(r.centroid_x - best->centroid_x) < 0.01
                        && std::abs(r.centroid_y - best->centroid_y) < 0.01);

    // SPHERE: biểu diễn frontier region centroid
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "map"; sphere.header.stamp = now;
    sphere.ns = "frontier_nodes"; sphere.id = id++;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = r.centroid_x;
    sphere.pose.position.y = r.centroid_y;
    sphere.pose.position.z = 0.05;
    sphere.pose.orientation.w = 1.0;
    if (is_best) {
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.35;
      sphere.color.r = 1.0f; sphere.color.g = 0.85f;
      sphere.color.b = 0.0f; sphere.color.a = 1.0f; // Vàng = best
    } else {
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.20;
      sphere.color.r = 0.0f; sphere.color.g = 0.85f;
      sphere.color.b = 1.0f; sphere.color.a = 0.85f; // Xanh dương
    }
    arr.markers.push_back(sphere);

    // TEXT_VIEW_FACING: label hiển thị tên + kích thước frontier
    visualization_msgs::msg::Marker text;
    text.header.frame_id = "map"; text.header.stamp = now;
    text.ns = "frontier_labels"; text.id = id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = r.centroid_x;
    text.pose.position.y = r.centroid_y;
    text.pose.position.z = 0.30;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.18;
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
    text.text = "F" + std::to_string(i) + "(" + std::to_string(r.size()) + "c)";
    if (is_best) text.text = "[BEST] " + text.text;
    arr.markers.push_back(text);
  }

  // ARROW: chỉ hướng từ robot → goal nếu có goal
  if (cached_has_goal_) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "map"; arrow.header.stamp = now;
    arrow.ns = "frontier_path"; arrow.id = id++;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.06; arrow.scale.y = arrow.scale.z = 0.12;
    arrow.color.r = 0.1f; arrow.color.g = 1.0f;
    arrow.color.b = 0.3f; arrow.color.a = 0.9f; // Xanh lá
    geometry_msgs::msg::Point s, e;
    s.x = robot_x; s.y = robot_y; s.z = 0.05;
    e.x = goal_x;   e.y = goal_y;   e.z = 0.05;
    arrow.points.push_back(s); arrow.points.push_back(e);
    arr.markers.push_back(arrow);
  }

  pub_frontier_markers_->publish(arr);
}

// ════════════════════════════════════════════════════════════════════════════
//  --------------------------------- A* GLOBAL PLANNER -----------------------
// ════════════════════════════════════════════════════════════════════════════

void AutonomousExploration::globalPlannerTimerCallback()
{
    if (!map_initialized_) return;
    if (!exploration_mode_.load()) return;

    double rx, ry, rth;
    try {
        auto tf = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time());
        rx  = tf.transform.translation.x;
        ry  = tf.transform.translation.y;
        tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                          tf.transform.rotation.z, tf.transform.rotation.w);
        rth = getYaw(q);
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[A*] Cannot get map→base TF: %s", ex.what());
        return;
    }

    slam_globalPlanner(rx, ry, rth);
}

void AutonomousExploration::slam_globalPlanner(double rx, double ry, double rth)
{
    // Dùng cached grid thay vì build 256K cells mới
    if (!cached_grid_valid_) return;

    // Nhận goal từ Frontier
    std::lock_guard<std::mutex> lock(map_mutex_);
    global_planner_.updateMap(cached_occ_grid_);

    if (frontier_explorer_.hasGoal()) {
        double gx = frontier_explorer_.getGoalX();
        double gy = frontier_explorer_.getGoalY();

        if (!active_goal_valid_ || std::hypot (gx - active_goal_x_, gy - active_goal_y_) > 0.05 )
        {
            global_planner_.setGoal(gx, gy);
            active_goal_x_ = gx;
            active_goal_y_ = gy;
            active_goal_valid_ = true;
            RCLCPP_INFO(get_logger(), "[A*] New goal from Fontier: (%.2f, %.2f)", gx, gy);
        }
    } else {
        if (!global_planner_.hasGoal()) return;
    }

    global_planner_.compute(rx, ry, rth, ++global_planner_step_);

    if (global_planner_.hasPath()) {
        const auto& path = global_planner_.getCurrentPath();
        cached_global_path_ = path.waypoints;
        cached_path_valid_  = true;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[A*] Path: %d waypoints | Next WP: (%.2f, %.2f) | GoalReached: %s",
            static_cast<int>(cached_global_path_.size()),
            global_planner_.getCurrentWaypointX(),
            global_planner_.getCurrentWaypointY(),
            global_planner_.isGoalReached() ? "YES" : "NO");
    } else {
        cached_path_valid_ = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[A*] No valid path (A* failed or no goal).");
    }

    if (global_planner_.isGoalReached()) {
        RCLCPP_INFO(get_logger(),
            "[A*] Goal (%.2f, %.2f) REACHED → signal Frontier.",
            frontier_explorer_.getGoalX(), frontier_explorer_.getGoalY());
        frontier_explorer_.signalGoalReached();
        global_planner_.reset();
        cached_path_valid_ = false;
    }

    // Xuất bản waypoints dưới dạng MarkerArray để hiển thị trên RViz
    publishGlobalPath();
}

//  publishAStarWaypoints — publish A* waypoints as MarkerArray on /autonomous_exploration/astar_waypoints
void AutonomousExploration::publishGlobalPath()
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp    = this->now();

    for (const auto& wp : cached_global_path_) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path_msg.header;
        ps.pose.position.x = wp.first;
        ps.pose.position.y = wp.second;
        ps.pose.position.z = 0.02;
        ps.pose.orientation.w = 1.0;
        path_msg.poses.push_back(ps);
    }

    pub_global_path_->publish(path_msg);

    publishAStarWaypoints();
}

void AutonomousExploration::publishAStarWaypoints()
{
    visualization_msgs::msg::MarkerArray arr;
    auto now = this->now();

    // Xóa tất cả markers cũ
    visualization_msgs::msg::Marker del_all;
    del_all.header.frame_id = "map";
    del_all.header.stamp    = now;
    del_all.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del_all);

    if (!cached_path_valid_ || cached_global_path_.empty()) {
        pub_astar_waypoints_->publish(arr);
        return;
    }

    int id = 0;

    // ── 1. Các waypoint phía trước (màu xanh dương) ──────────────────────
    int current_wp = global_planner_.hasPath() ? 0 : 0;

    // Tìm current_wp_idx từ global_planner_ (không có getter public)
    // Ta ước lượng bằng cách tìm waypoint gần robot nhất
    double rx = 0.0, ry = 0.0;
    try {
        auto tf = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time());
        rx = tf.transform.translation.x;
        ry = tf.transform.translation.y;
    } catch (...) { /* ignore */ }

    // Tìm waypoint gần robot nhất
    int nearest_idx = 0;
    double nearest_dist = 1e9;
    for (size_t i = 0; i < cached_global_path_.size(); ++i) {
        double d = std::hypot(cached_global_path_[i].first - rx,
                              cached_global_path_[i].second - ry);
        if (d < nearest_dist) {
            nearest_dist = d;
            nearest_idx = static_cast<int>(i);
        }
    }
    current_wp = nearest_idx;

    for (size_t i = 0; i < cached_global_path_.size(); ++i) {
        const auto& wp = cached_global_path_[i];

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp    = now;
        marker.ns = "astar_waypoints";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = wp.first;
        marker.pose.position.y = wp.second;
        marker.pose.position.z = 0.08;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.12;
        marker.color.a = 0.85f;

        if (static_cast<int>(i) == current_wp) {
            // Waypoint hiện tại: màu xanh lá
            marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.20; // kích thước waypoint
        } else if (i == cached_global_path_.size() - 1) { 
            // Waypoint cuối cùng (goal): màu đỏ
            marker.color.r = 1.0f; marker.color.g = 0.2f; marker.color.b = 0.2f;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.22;
        } else if (static_cast<int>(i) < current_wp) {
            // Waypoint đã đi qua: màu xám mờ
            marker.color.r = 0.5f; marker.color.g = 0.5f; marker.color.b = 0.5f;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.08;
            marker.color.a = 0.4f;
        } else {
            // Waypoint chưa đi qua: màu xanh dương
            marker.color.r = 0.2f; marker.color.g = 0.5f; marker.color.b = 1.0f;
        }

        arr.markers.push_back(marker);

        // Thêm label TEXT_VIEW_FACING cho các waypoint quan trọng
        if (static_cast<int>(i) == current_wp || i == cached_global_path_.size() - 1 || i % 10 == 0) {
            visualization_msgs::msg::Marker text;
            text.header.frame_id = "map";
            text.header.stamp    = now;
            text.ns = "astar_labels";
            text.id = id++;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.position.x = wp.first;
            text.pose.position.y = wp.second;
            text.pose.position.z = 0.35;
            text.pose.orientation.w = 1.0;
            text.scale.z = 0.14;
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;

            if (static_cast<int>(i) == current_wp) {
                text.text = "[CURRENT] WP" + std::to_string(i);
                text.color.g = 1.0f; // xanh lá
            } else if (i == cached_global_path_.size() - 1) {
                text.text = "[GOAL] WP" + std::to_string(i);
                text.color.r = 1.0f; // đỏ
            } else {
                text.text = "WP" + std::to_string(i);
            }
            arr.markers.push_back(text);
        }
    }

    pub_astar_waypoints_->publish(arr);
}

// ════════════════════════════════════════════════════════════════════════════
//  --------------------------------- DWA LOCAL PLANNER ----------------------
// ════════════════════════════════════════════════════════════════════════════

void AutonomousExploration::localPlannerTimerCallback()
{
    if (!map_initialized_)         return;
    if (!exploration_mode_.load()) return;
    if (!cached_path_valid_) 
    {
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.angular.z = 0.0;
        pub_cmd_->publish(stop);
        current_v_ = 0.0;
        current_w_ = 0.0;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                "[DWA] No valid path form A* => Robot Stop...");
        return;
    }
    if (cached_global_path_.empty()) return;

    double rx, ry, rth;
    try {
        auto tf = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time());
        rx  = tf.transform.translation.x;
        ry  = tf.transform.translation.y;
        tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                          tf.transform.rotation.z, tf.transform.rotation.w);
        rth = getYaw(q);
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[DWA] Cannot get map→base TF: %s", ex.what());
        return;
    }

    slam_localPlanner(rx, ry, rth, current_v_, current_w_);
}

void AutonomousExploration::slam_localPlanner(double rx, double ry, double rth,
                                               double rv, double rw)
{
    if (cached_scan_ranges_.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "[DWA] No LaserScan data yet — skipping.");
        return;
    }

    // ================================================================
    //  STUCK DETECTION: nếu robot ra lệnh nhưng không di chuyển
    //  trong STUCK_TIME_THRESH giây → kích hoạt recovery
    // ================================================================
    if (!stuck_detected_)
    {
        // Tính khoảng cách di chuyển từ lần kiểm tra trước
        double dx = rx - stuck_prev_x_;
        double dy = ry - stuck_prev_y_;
        double dist_moved = std::sqrt(dx * dx + dy * dy);

        // Nếu robot đang ra lệnh chạy (v != 0 hoặc w != 0)
        // nhưng di chuyển rất ít → cộng dồn thời gian kẹt
        if (std::abs(rv) > 0.01 || std::abs(rw) > 0.02)
        {
            if (dist_moved < STUCK_DIST_THRESH)
            {
                stuck_movement_timer_ += 0.2; // 200ms mỗi lần gọi
                if (stuck_movement_timer_ > STUCK_TIME_THRESH)
                {
                    RCLCPP_WARN(get_logger(),
                        "[STUCK] Detected! Robot not moving for %.1fs despite commands. "
                        "dist_moved=%.3f v=%.3f w=%.3f → Starting recovery...",
                        stuck_movement_timer_, dist_moved, rv, rw);
                    stuck_detected_ = true;
                    stuck_phase_ = 1; // Bắt đầu phase lùi
                    stuck_escape_elapsed_ = 0.0;
                }
            }
            else
            {
                // Robot di chuyển bình thường → reset timer
                stuck_movement_timer_ = 0.0;
            }
        }
        else
        {
            // Robot đang dừng (không có lệnh) → không detect kẹt
            stuck_movement_timer_ = 0.0;
        }

        stuck_prev_x_ = rx;
        stuck_prev_y_ = ry;
    }

    // ================================================================
    //  STUCK RECOVERY: override cmd_vel nếu đang ở chế độ thoát kẹt
    // ================================================================
    if (stuck_detected_)
    {
        geometry_msgs::msg::Twist escape_cmd;
        stuck_escape_elapsed_ += 0.2; // 200ms mỗi lần gọi

        if (stuck_phase_ == 1)
        {
            // Phase 1: Lùi nhẹ để tạo khoảng trống
            escape_cmd.linear.x = STUCK_REVERSE_V;
            escape_cmd.angular.z = 0.0;
            if (stuck_escape_elapsed_ >= STUCK_REVERSE_TIME)
            {
                // Phase 2: Xoay về phía trống hơn
                // Dùng LiDAR để chọn hướng xoay
                stuck_phase_ = 2;
                stuck_escape_elapsed_ = 0.0;
                RCLCPP_INFO(get_logger(), "[STUCK] Phase 1 done → Phase 2: Turn");
            }
        }
        else if (stuck_phase_ == 2)
        {
            double left_clearance = 0.0, right_clearance = 0.0;
            if (!cached_scan_ranges_.empty())
            {
                // Tính clearance trái/phải từ scan
                size_t n = cached_scan_ranges_.size();
                double angle_min = cached_scan_angle_min_;
                double angle_inc = cached_scan_angle_increment_;
                for (size_t i = 0; i < n; ++i)
                {
                    double r = cached_scan_ranges_[i];
                    if (std::isinf(r) || std::isnan(r) || r < 0.12) continue;
                    double angle = normalizeAngle(angle_min + i * angle_inc);
                    if (angle > 0 && angle < M_PI_2)
                        left_clearance += std::min(r, 3.0);
                    else if (angle < 0 && angle > -M_PI_2)
                        right_clearance += std::min(r, 3.0);
                }
            }

            // Xoay về hướng có clearance lớn hơn, mặc định xoay trái
            escape_cmd.linear.x = 0.0;
            if (left_clearance > right_clearance + 1.0)
                escape_cmd.angular.z = STUCK_TURN_W;  // Xoay trái
            else if (right_clearance > left_clearance + 1.0)
                escape_cmd.angular.z = -STUCK_TURN_W; // Xoay phải
            else
                escape_cmd.angular.z = STUCK_TURN_W;  // Mặc định xoay trái

            if (stuck_escape_elapsed_ >= STUCK_TURN_TIME)
            {
                stuck_phase_ = 3;
                stuck_escape_elapsed_ = 0.0;
                RCLCPP_INFO(get_logger(), "[STUCK] Phase 2 done → Phase 3: Recovery wait");
            }
        }
        else if (stuck_phase_ == 3)
        {
            // Phase 3: Chờ hồi phục, cho DWA hoạt động lại
            escape_cmd.linear.x = 0.0;
            escape_cmd.angular.z = 0.0;
            if (stuck_escape_elapsed_ >= STUCK_RECOVERY_TIME)
            {
                RCLCPP_INFO(get_logger(), "[STUCK] Recovery complete → Resume normal control");
                stuck_detected_ = false;
                stuck_phase_ = 0;
                stuck_movement_timer_ = 0.0;
            }
        }

        pub_cmd_->publish(escape_cmd);
        current_v_ = escape_cmd.linear.x;
        current_w_ = escape_cmd.angular.z;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "[STUCK] Phase %d: v=%.3f w=%.3f elapsed=%.1fs",
            stuck_phase_, escape_cmd.linear.x, escape_cmd.angular.z,
            stuck_escape_elapsed_);
        return; // Bỏ qua DWA khi đang escape
    }

    DWAState state(rx, ry, rth, rv, rw);

    std::vector<float> scan_f(cached_scan_ranges_.begin(), cached_scan_ranges_.end());

    auto [v_star, w_star] = dwa_planner_.computeVelocity(
        state,
        scan_f,
        cached_scan_angle_min_,
        cached_scan_angle_increment_,
        0.0, // yaw_offset: LiDAR đặt thẳng trục robot
        rx, ry, rth,
        cached_global_path_ 
    );

    current_v_ = v_star;
    current_w_ = w_star;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = v_star;
    cmd.angular.z = w_star;
    pub_cmd_->publish(cmd);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "[DWA] cmd_vel → v=%.3f m/s  w=%.3f rad/s | Path WPs: %zu",
        v_star, w_star, cached_global_path_.size());
}


// ════════════════════════════════════════════════════════════════════════════
//  main
// ════════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);                                                // Khởi tạo ROS2
    auto node = std::make_shared<AutonomousExploration>();                   // Gọi hàm Constructor
    rclcpp::executors::MultiThreadedExecutor executor;                       // Executor đa luồng
    executor.add_node(node);                                                 // Thêm node vào executor
    executor.spin();                                                         // Bắt đầu vòng lặp xử lý callback
    rclcpp::shutdown();                                                      // /rosout
    return 0;
}