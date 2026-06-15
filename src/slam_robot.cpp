#include "include/slam_robot.h"

using std::placeholders::_1;

// ════════════════════════════════════════════════════════════════════════════
//  Constructor
// ════════════════════════════════════════════════════════════════════════════
SlamRobot::SlamRobot() : Node("slam_robot") {
    // ── Callback groups ───────────────────────────────────────────────────
    callback_group_lidar_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // lidar
    callback_group_slam_  =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // slam

    // ── TF2 ──────────────────────────────────────────────────────────────
    tf_buffer_      = std::make_shared<tf2_ros::Buffer>(this->get_clock());      
    tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ── Publishers ────────────────────────────────────────────────────────
    pub_map_    = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local());                             // map

    pub_graph_nodes_ = create_publisher<geometry_msgs::msg::PoseArray>(
        "/slam_robot/graph_nodes", rclcpp::QoS(5).transient_local());          // PoseArray

    pub_graph_edges_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/slam_robot/graph_edges", rclcpp::QoS(5).transient_local());          // MarkerArray

    pub_loop_closure_event_ = create_publisher<std_msgs::msg::String>(
        "/slam_robot/loop_closure_event", rclcpp::QoS(5).transient_local());   // Loop closure events

    pub_scan_visualization_ = create_publisher<sensor_msgs::msg::LaserScan>(
        "/slam_robot/scan_visualization", rclcpp::QoS(10).transient_local());  // LiDAR scan visualization

    // ── MapBuilder: 32m × 20m grid, resolution 5cm, origin (-17,-10) ─────
    map_builder_ = slam::MapBuilder(0.05, 640, 400, -17.0, -10.0);
    map_builder_.setPublisher(pub_map_);

    // ── Link MapBuilder to SlamGraph ──────────────────────────────────────
    slam_graph_.setMapBuilder(&map_builder_);                                   // Kết nối SlamGraph với MapBuilder
    slam_graph_.init();                                                         // Khởi tạo graph với 1 node anchor (0,0,0)

    // ── Subscriber ────────────────────────────────────────────────────────
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_lidar_;
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/agv_scan", rclcpp::SensorDataQoS(),
        std::bind(&SlamRobot::scanCallback, this, _1), sub_opt);                // LiDAR scan ( 10Hz )

    // ── Timers ────────────────────────────────────────────────────────────
    // 200ms = 5 Hz: get TF pose → slamTimerCallback → publish graph visualization
    timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&SlamRobot::slamTimerCallback, this),
        callback_group_slam_);

    // 20ms = 50 Hz: TF broadcast (decoupled from SLAM — giúp Rviz mượt)
    tf_broadcast_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() { broadcastMapOdomTF(this->now()); });

    // 500ms = 2 Hz: publish map
    map_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&SlamRobot::mapBuilderTimerCallback, this));

    // 2000ms = 0.5 Hz: graph visualization (chỉ publish khi có thay đổi)
    graph_viz_timer_ = create_wall_timer(
        std::chrono::milliseconds(2000),
        std::bind(&SlamRobot::graphVizTimerCallback, this));

    RCLCPP_INFO(get_logger(),
        "[SlamRobot] Graph-Based SLAM initialized (Grisetti et al., 2010).");
    RCLCPP_INFO(get_logger(),
        "[SlamRobot] Library: lib/SLAM_Graph_Based | Optimizer: Gauss-Newton");
}

// ════════════════════════════════════════════════════════════════════════════
//  ---------------------------- SLAM GRAPH BASED -----------------------------
// ════════════════════════════════════════════════════════════════════════════
// ════════════════════════════════════════════════════════════════════════════
//  Helper: broadcast map→odom TF (corrected by Graph SLAM)
// ════════════════════════════════════════════════════════════════════════════
void SlamRobot::broadcastMapOdomTF(rclcpp::Time now)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";
    tf.transform.translation.x = map_odom_x; 
    tf.transform.translation.y = map_odom_y;  
    tf.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, map_odom_theta);            
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
}

//  slamTimerCallback — 5 Hz: tính pose + SLAM + broadcast map→odom
void SlamRobot::slamTimerCallback()
{
    // 1. Lấy pose odom→base_link
    double ox, oy, otheta; // odom → base_link
    try{
        auto tf_odom = tf_buffer_->lookupTransform("odom", "base_link", rclcpp::Time());
        ox = tf_odom.transform.translation.x;
        oy = tf_odom.transform.translation.y;
        // Chuyển quaternion sang góc yaw (θ) 
        tf2::Quaternion q(tf_odom.transform.rotation.x,
                          tf_odom.transform.rotation.y,
                          tf_odom.transform.rotation.z,
                          tf_odom.transform.rotation.w);
        otheta = getYaw(q);
    }
    catch (tf2::TransformException& ex){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[TF] Cannot get odom→base TF: %s", ex.what());
        return;
    }

    // 2. Tính map→base_link = map_odom ⊕ odom_base (composition)
    double mx = map_odom_x + cos(map_odom_theta) * ox - sin(map_odom_theta) * oy;
    double my = map_odom_y + sin(map_odom_theta) * ox + cos(map_odom_theta) * oy;
    double mtheta = normalizeAngle(map_odom_theta + otheta);

    // 3. Thực hiện Graph-Based SLAM với pose trong map frame
    // slam_result: -1 = không có node, 0 = có node mới, 1 = có node mới + optimized
    int slam_result = graphSLAMcall(mx, my, mtheta);
    
    // 4. Nếu có node mới → cập nhật map→odom TF
    if (slam_result >= 0) {
        int last = slam_graph_.pose_graph.numNodes() - 1;
        if (last >= 1) { // có ít nhất node 0 (anchor) + node mới
            const auto& n = slam_graph_.pose_graph.nodes[last];
            // map_odom = optimized_map_base * inverse(odom_base)
            updateMapOdom(n.x, n.y, n.theta, ox, oy, otheta);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[SLAM] map→odom: (%.3f, %.3f, %.3f°) | raw=(%.3f,%.3f) | opt=(%.3f,%.3f)",
                map_odom_x, map_odom_y, map_odom_theta * 180.0 / M_PI,
                ox, oy, n.x, n.y);
        }
    }

    // Lưu ý: map→odom TF được broadcast ở 50Hz bởi tf_broadcast_timer_
}

// ── Sau optimize: cập nhật map→odom ──────────────────────────
// P_map_odom = P_map_base * inverse(P_odom_base)
void SlamRobot::updateMapOdom(double map_x, double map_y, double map_theta,
                                double odom_x, double odom_y, double odom_theta)
{
    // inverse(odom_base): ( -cos(θ)*x - sin(θ)*y, sin(θ)*x - cos(θ)*y, -θ )
    double inv_x = -cos(odom_theta) * odom_x - sin(odom_theta) * odom_y; // = -cos(θ)*ox - sin(θ)*oy
    double inv_y = sin(odom_theta) * odom_x - cos(odom_theta) * odom_y; // =  sin(θ)*ox - cos(θ)*oy
    // Tích: map_odom = map_base * inv(odom_base)
    map_odom_x = map_x + cos(map_theta) * inv_x - sin(map_theta) * inv_y;
    map_odom_y = map_y + sin(map_theta) * inv_x + cos(map_theta) * inv_y;
    map_odom_theta = normalizeAngle(map_theta - odom_theta);
}

// scanCallback - 10Hz : LiDAR Thread
void SlamRobot::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Bắt LiDAR scan để sử dụng vào hàm graphSLAMcall (odometry + loop closure)
    cached_scan_ranges_.assign(scan->ranges.begin(), scan->ranges.end());
    cached_scan_angle_min_       = scan->angle_min;
    cached_scan_angle_increment_ = scan->angle_increment;

    // Log khoảng cách 8 hướng của Lidar ( Front, Left, Right,...)
    double dists[8] = {99, 99, 99, 99, 99, 99, 99, 99};
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double r = scan->ranges[i];
        if (std::isnan(r) || r < 0.12) continue;
        double deg = (scan->angle_min + i * scan->angle_increment) * 180.0 / M_PI;
        while (deg <   0) deg += 360;
        while (deg >= 360) deg -= 360;
        int sector = static_cast<int>((deg + 22.5) / 45.0) % 8;
        if (r < dists[sector]) dists[sector] = r;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "[DIST] F:%.1f FL:%.1f L:%.1f BL:%.1f B:%.1f BR:%.1f R:%.1f FR:%.1f",
        dists[0], dists[1], dists[2], dists[3], dists[4], dists[5], dists[6], dists[7]);

    // Publish scan visualization — giảm tần suống 5Hz (skip 1 frame)
    static int scan_pub_cnt = 0;
    if (++scan_pub_cnt % 2 == 0) {
        pub_scan_visualization_->publish(*scan);
    }

    // Cập nhật occupanc grid từ LiDAR
    try {
        // Lấy odom→base_link tại đúng thời điểm scan (chính xác về mặt thời gian)
         auto tf_odom = tf_buffer_->lookupTransform("odom", "base_link", rclcpp::Time(scan->header.stamp));
        double ox = tf_odom.transform.translation.x;
        double oy = tf_odom.transform.translation.y;
        tf2::Quaternion q_odom(tf_odom.transform.rotation.x,
                               tf_odom.transform.rotation.y,
                               tf_odom.transform.rotation.z,
                               tf_odom.transform.rotation.w);
        // Lấy hướng quay của robot (yaw) dùng helper getYaw()
        double otheta = getYaw(q_odom);

        // Tính map→base_link = map_odom ⊕ odom_base (composition)
        // Dùng map_odom_* (hiệu chỉnh SLAM mới nhất) + odom→base ở thời điểm scan
        double px = map_odom_x + cos(map_odom_theta) * ox - sin(map_odom_theta) * oy;
        double py = map_odom_y + sin(map_odom_theta) * ox + cos(map_odom_theta) * oy;
        double yaw = normalizeAngle(map_odom_theta + otheta);

        // Lấy dữ liệu khoảng cách từ LiDAR
        std::vector<float> ranges_f(scan->ranges.begin(), scan->ranges.end());

        // Cập nhật occupancy grid
        map_builder_.updateFromRanges(ranges_f, scan->angle_min, scan->angle_increment,
                                      px, py, yaw);

        if (!map_initialized_) {
            map_initialized_ = true;
            RCLCPP_INFO(get_logger(), "[MapBuilder] Map initialized after first successful LiDAR scan.");
        }
    }
    catch (tf2::TransformException& ex){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[TF] Cannot update map from scan: %s", ex.what());
    }
}

// ════════════════════════════════════ graphSLAMcall ════════════════════════════════════
//  
//  Thực hiện đầy đủ quy trình Graph-Based SLAM gồm 4 bước:
//  Step 1 — Front-End (Odometry):
//    δx = R_{i-1}^T · (t_i − t_{i-1}),  δθ = normalize(θ_i − θ_{i-1})
//    Gọi addOdometryNode():
//       - Thêm node pose mới x_i vào đồ thị.
//       - Thêm cạnh odometry (i−1, i, z_odom, Ω_odom).
//
//  Step 2 — Front-End (Loop Closure):
//    Proximity filter: ‖t_j − t_i‖ < d_thresh && |θ_j − θ_i| < θ_thresh
//    Scan correlation: C = Σ r_i·r_j / (‖r_i‖·‖r_j‖) > corr_thresh
//    → Gọi addLoopClosures(): Nếu hai scan được xác định là khớp,
//    thêm cạnh đóng vòng lặp (i, j, z_scan, Ω_loop) vào đồ thị.
//
//  Step 3 — Back-End (Gauss-Newton Optimization):
//    x* = argmin Σ_{<i,j>∈E} e_ij^T · Ω_ij · e_ij
//    1. Xây dựng ma trận Hessian H.
//    2. Xây dựng vector gradient b.
//    3. Thiết lập hệ:
//          H · Δξ = −b
//    4. Giải hệ bằng phương pháp khử Gauss (Gaussian Elimination).
//    5. Cập nhật toàn bộ pose trong đồ thị:
//          x ← x ⊞ Δξ
//
//  Step 4 — Map Rebuild (if optimized):
//    clearMap() → updateMapFromNode(i)  với mọi node i trong pose graph (Log-Odds ray-casting)
//    Bản đồ Occupancy Grid được xây dựng lại bằng:
//      - Biểu diễn Log-Odds
//      - Bresenham Ray-Casting
//      - Dữ liệu LiDAR đã lưu trong từng node
// ════════════════════════════════════════════════════════════════════════════
// ════════════════════════════════════════════════════════════════════════════
//  graphSLAMcall — Executes one full Graph-Based SLAM cycle
// ════════════════════════════════════════════════════════════════════════════
int SlamRobot::graphSLAMcall(double x, double y, double theta) {
    // ── Step 1: Add odometry node ─────────────────────────────────────────
    int new_idx = slam_graph_.addOdometryNode(
        x, y, theta,
        cached_scan_ranges_,
        cached_scan_angle_min_,
        cached_scan_angle_increment_);

    if (new_idx < 0) return -1;  // Nếu robot chưa di chuyển thì return luôn, không thêm node mới

    // ── Step 2: Detect loop closures ──────────────────────────────────────
    int matched_id = slam_graph_.addLoopClosures(new_idx);
    if (matched_id >= 0) {
        std_msgs::msg::String msg;
        msg.data = "Loop closure: Node[" + std::to_string(matched_id) +
                   "] <-> Node[" + std::to_string(new_idx) +
                   "] | Total: " + std::to_string(slam_graph_.loop_closure_count_);
        pub_loop_closure_event_->publish(msg);
        RCLCPP_INFO(get_logger(), "[SLAM] %s", msg.data.c_str());
    }

    // ── Step 3: Gauss-Newton back-end optimization ────────────────────────
    bool optimized = slam_graph_.optimizeIfNeeded();

    // ── Step 4: Log optimization result ───────────────────────────────────
    if (optimized) {
        RCLCPP_WARN(get_logger(), "[SLAM] Optimization completed | Nodes: %d",
            slam_graph_.pose_graph.numNodes());
    }

    // ── Đánh dấu graph đã thay đổi ──────────────────────────────────────
    graph_has_changed_ = true;
    graph_viz_dirty_.store(true);

    return optimized ? 1 : 0;
}

// ════════════════════════════════════════════════════════════════════════════
//  mapBuilderTimerCallback — 2 Hz: publish OccupancyGrid (dùng cached grid)
// ════════════════════════════════════════════════════════════════════════════
void SlamRobot::mapBuilderTimerCallback() {
    if (!map_initialized_) return;
    // Build fresh grid, cache it, then publish
    refreshCachedGrid();
    std::lock_guard<std::mutex> lock(map_mutex_);
    pub_map_->publish(cached_occ_grid_);
}

// ════════════════════════════════════════════════════════════════════════════
//  refreshCachedGrid — Build a fresh OccupancyGrid snapshot under mutex
// ════════════════════════════════════════════════════════════════════════════
void SlamRobot::refreshCachedGrid() {
    auto fresh = map_builder_.buildOccupancyGrid(this->now());
    std::lock_guard<std::mutex> lock(map_mutex_);
    cached_occ_grid_ = std::move(fresh);
    cached_grid_valid_ = true;
}

// ════════════════════════════════════════════════════════════════════════════
//  graphVizTimerCallback — 0.5 Hz: publish graph visualization only when dirty
// ════════════════════════════════════════════════════════════════════════════
void SlamRobot::graphVizTimerCallback()
{
    if (!graph_viz_dirty_.load()) return;  // Nếu không thay đổi thì skip
    graph_viz_dirty_.store(false);

    const int N = slam_graph_.pose_graph.numNodes();
    if (N == 0) return;

    // /slam_robot/graph_nodes — PoseArray
    geometry_msgs::msg::PoseArray node_msg;
    node_msg.header.frame_id = "map";
    node_msg.header.stamp    = this->now();

    for (int i = 0; i < N; ++i) {
        const auto& gn = slam_graph_.pose_graph.nodes[i];
        geometry_msgs::msg::Pose p;
        p.position.x = gn.x;
        p.position.y = gn.y;
        p.position.z = 0.05;
        tf2::Quaternion q;
        q.setRPY(0, 0, gn.theta);
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();
        node_msg.poses.push_back(p);
    }
    pub_graph_nodes_->publish(node_msg);

    // /slam_robot/graph_edges — MarkerArray
    visualization_msgs::msg::MarkerArray edge_msg;
    visualization_msgs::msg::Marker del_all;
    del_all.header.frame_id = "map";
    del_all.header.stamp    = node_msg.header.stamp;
    del_all.action = visualization_msgs::msg::Marker::DELETEALL;
    edge_msg.markers.push_back(del_all);

    visualization_msgs::msg::Marker odom_lines, loop_lines;
    auto initLine = [&](visualization_msgs::msg::Marker& m,
                        const std::string& ns, int id,
                        float r, float g, float b, float a, float w)
    {
        m.header.frame_id = "map";
        m.header.stamp    = node_msg.header.stamp;
        m.ns = ns;
        m.id = id;
        m.type   = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = w;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = a;
    };
    initLine(odom_lines, "odom_edges", 0, 0.3f, 0.3f, 1.0f, 0.6f, 0.02f);
    initLine(loop_lines, "loop_edges", 1, 1.0f, 0.2f, 0.2f, 0.9f, 0.04f);

    for (const auto& e : slam_graph_.pose_graph.edges) {
        if (e.from >= N || e.to >= N) continue;
        geometry_msgs::msg::Point p1, p2;
        p1.x = slam_graph_.pose_graph.nodes[e.from].x;
        p1.y = slam_graph_.pose_graph.nodes[e.from].y;
        p1.z = 0.02;
        p2.x = slam_graph_.pose_graph.nodes[e.to].x;
        p2.y = slam_graph_.pose_graph.nodes[e.to].y;
        p2.z = 0.02;
        if (e.is_loop) {
            loop_lines.points.push_back(p1);
            loop_lines.points.push_back(p2);
        } else {
            odom_lines.points.push_back(p1);
            odom_lines.points.push_back(p2);
        }
    }

    edge_msg.markers.push_back(odom_lines);
    edge_msg.markers.push_back(loop_lines);
    pub_graph_edges_->publish(edge_msg);
}

// ════════════════════════════════════════════════════════════════════════════
//  main
// ════════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);                           // Khởi tạo ROS2
    auto node = std::make_shared<SlamRobot>();          // Gọi hàm Constructor
    rclcpp::executors::MultiThreadedExecutor executor;  // Executor đa luồng
    executor.add_node(node);                            // Thêm node vào executor
    executor.spin();                                    // Bắt đầu vòng lặp xử lý callback
    rclcpp::shutdown();                                 // /rosout
    return 0;
}
