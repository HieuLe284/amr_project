#include "include/slam_robot.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;

// ════════════════════════════════════════════════════════════════════════════
//  Helper: broadcast map→odom TF (identity — map = odom at origin)
// ════════════════════════════════════════════════════════════════════════════
static void broadcastMapOdomTF(
    rclcpp::Time now,
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = now;
    tf.header.frame_id = "map";
    tf.child_frame_id  = "odom";
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf2::Quaternion q;  q.setRPY(0, 0, 0);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    broadcaster->sendTransform(tf);
}

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
    pub_cmd_    = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); // moving

    pub_map_    = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local());                             // map

    pub_graph_nodes_ = create_publisher<geometry_msgs::msg::PoseArray>(
        "/slam_robot/graph_nodes", rclcpp::QoS(5).transient_local());          // PoseArray

    pub_graph_edges_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/slam_robot/graph_edges", rclcpp::QoS(5).transient_local());          // MarkeyArray

    pub_loop_closure_event_ = create_publisher<std_msgs::msg::String>(
        "/slam_robot/loop_closure_event", rclcpp::QoS(5).transient_local());   // Loop closure events

    pub_scan_visualization_ = create_publisher<sensor_msgs::msg::LaserScan>(
        "/slam_robot/scan_visualization", rclcpp::QoS(10).transient_local());  // LiDAR scan visualization

    // ── MapBuilder: 32m × 20m grid, resolution 5cm, origin (-17,-10) ─────
    map_builder_ = slam::MapBuilder(0.05, 640, 400, -17.0, -10.0);
    map_builder_.setPublisher(pub_map_);

    // ── Link MapBuilder to SlamGraph ──────────────────────────────────────
    slam_graph_.setMapBuilder(&map_builder_);                                   // kết nối SlamGraph với MapBuilder
    slam_graph_.init();                                                         //  Khởi tạo graph với 1 node anchor (0,0,0)

    // ── Subscriber ────────────────────────────────────────────────────────
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_lidar_;
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/agv_scan", rclcpp::SensorDataQoS(),
        std::bind(&SlamRobot::scanCallback, this, _1), sub_opt);                // LiDAR scan

    // ── Timers ────────────────────────────────────────────────────────────
    // 200ms = 5 Hz: get TF pose → slamTimerCallback → publish graph visualization
    timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&SlamRobot::slamTimerCallback, this),
        callback_group_slam_);

    // 200ms = 5 Hz: publish mapBuilderTimerCallback
    map_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&SlamRobot::mapBuilderTimerCallback, this));

    RCLCPP_INFO(get_logger(),
        "[SlamRobot] Graph-Based SLAM initialized (Grisetti et al., 2010).");
    RCLCPP_INFO(get_logger(),
        "[SlamRobot] Library: lib/SLAM_Graph_Based | Optimizer: Gauss-Newton");
}

// ════════════════════════════════════════════════════════════════════════════
//  SLAM Graph Based
// ════════════════════════════════════════════════════════════════════════════
void SlamRobot::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    broadcastMapOdomTF(this->now(), tf_broadcaster_);

    // Cache LiDAR scan for use in graphSLAMcall (odometry + loop closure)
    cached_scan_ranges_.assign(scan->ranges.begin(), scan->ranges.end());
    cached_scan_angle_min_       = scan->angle_min;
    cached_scan_angle_increment_ = scan->angle_increment;

    // Log 8-direction distances ( Front, Left, Right,...)
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

    // Publish scan visualization
    pub_scan_visualization_->publish(*scan);

    // Mỗi 2s gọi map_builder 1 lần
    static int scan_counter = 0;
    if (++scan_counter % 2 == 0) {
        auto occ = map_builder_.buildOccupancyGrid(this->now());
        if (!map_initialized_) { // nếu map chưa init 
            map_initialized_ = true; 
            RCLCPP_INFO(get_logger(), "[MapBuilder] Map ready: %dx%d @ %.3fm",
                occ.info.width, occ.info.height, occ.info.resolution);
        }
    }

    // Cập nhật occupanc grid từ LiDAR
    try {
        auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        double px = tf.transform.translation.x;
        double py = tf.transform.translation.y;
        tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                          tf.transform.rotation.z, tf.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        std::vector<float> ranges_f(scan->ranges.begin(), scan->ranges.end());
        map_builder_.updateFromRanges(ranges_f, scan->angle_min, scan->angle_increment,
                                    px, py, yaw);
    }
    catch (tf2::TransformException& ex){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[TF] Cannot update map from scan: %s", ex.what());
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  slamTimerCallback — 5 Hz: get TF, call graphSLAMcall
// ════════════════════════════════════════════════════════════════════════════
void SlamRobot::slamTimerCallback() {
    // Broadcast TF map->odom
    broadcastMapOdomTF(this->now(), tf_broadcaster_);

    double x = 0.0, y = 0.0, theta = 0.0;
    try {
        auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        x = tf.transform.translation.x;
        y = tf.transform.translation.y;
        tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                          tf.transform.rotation.z, tf.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, theta);
    } catch (tf2::TransformException& ex) {
        // Nếu không có TF thì warning 5s rồi return
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[TF] Cannot get robot pose: %s", ex.what());
        return;
    }

    // nếu có TF thì gọi graphSLAMcall để xử lý SLAM
    if (rclcpp::ok())
        graphSLAMcall(x, y, theta);
}

// ════════════════════════════════════════════════════════════════════════════
//  graphSLAMcall — THE single SLAM processing function
//
//  Runs the complete 4-step Graph-Based SLAM pipeline:
//
//  Step 1 — Front-End (Odometry):
//    δx = R_{i-1}^T · (t_i − t_{i-1}),  δθ = normalize(θ_i − θ_{i-1})
//    → addOdometryNode: new node x_i + edge (i-1, i, z_odom, Ω_odom)
//
//  Step 2 — Front-End (Loop Closure):
//    Proximity filter: ‖t_j − t_i‖ < d_thresh && |θ_j − θ_i| < θ_thresh
//    Scan correlation: C = Σ r_i·r_j / (‖r_i‖·‖r_j‖) > corr_thresh
//    → addLoopClosures: new edge (i, j, z_scan, Ω_loop) if matched
//
//  Step 3 — Back-End (Gauss-Newton Optimization):
//    x* = argmin Σ_{<i,j>∈E} e_ij^T · Ω_ij · e_ij
//    Build H·Δξ = -b, solve via Gaussian elimination, update all poses
//
//  Step 4 — Map Rebuild (if optimized):
//    clearMap() → updateMapFromNode(i) for all i (Log-Odds ray-casting)
// ════════════════════════════════════════════════════════════════════════════
// ════════════════════════════════════════════════════════════════════════════
//  scanCallback — Cache LiDAR data, update map, publish scan viz
// ════════════════════════════════════════════════════════════════════════════
void SlamRobot::graphSLAMcall(double x, double y, double theta) {
    // ── Step 1: Add odometry node ─────────────────────────────────────────
    int new_idx = slam_graph_.addOdometryNode(
        x, y, theta,
        cached_scan_ranges_,
        cached_scan_angle_min_,
        cached_scan_angle_increment_);

    if (new_idx < 0) return;  // Nếu robot chưa di chuyển thì return luôn, không thêm node mới

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

    // ── Step 4: Rebuild map from all optimized poses ──────────────────────
    if (optimized) {
        slam_graph_.clearMap(); // xóa toàn bộ occupancy grid
        const int N = slam_graph_.pose_graph.numNodes();
        for (int i = 1; i < N; ++i)
            slam_graph_.updateMapFromNode(i);
        RCLCPP_WARN(get_logger(),
            "[SLAM] Map rebuilt from %d optimized nodes", N);
    }

    // ── Publish graph visualization (PoseArray + MarkerArray) ─────────────
    const int N = slam_graph_.pose_graph.numNodes();

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
        tf2::Quaternion q;  q.setRPY(0, 0, gn.theta);
        p.orientation.x = q.x();  p.orientation.y = q.y();
        p.orientation.z = q.z();  p.orientation.w = q.w();
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
        m.ns = ns;  m.id = id;
        m.type   = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = w;
        m.color.r = r;  m.color.g = g;  m.color.b = b;  m.color.a = a;
    };
    initLine(odom_lines, "odom_edges", 0, 0.3f, 0.3f, 1.0f, 0.6f, 0.02f);
    initLine(loop_lines, "loop_edges", 1, 1.0f, 0.2f, 0.2f, 0.9f, 0.04f);

    for (const auto& e : slam_graph_.pose_graph.edges) {
        if (e.from >= N || e.to >= N) continue;
        geometry_msgs::msg::Point p1, p2;
        p1.x = slam_graph_.pose_graph.nodes[e.from].x;
        p1.y = slam_graph_.pose_graph.nodes[e.from].y; p1.z = 0.02;
        p2.x = slam_graph_.pose_graph.nodes[e.to].x;
        p2.y = slam_graph_.pose_graph.nodes[e.to].y;   p2.z = 0.02;
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
//  mapBuilderTimerCallback — 5 Hz: publish OccupancyGrid
// ════════════════════════════════════════════════════════════════════════════
void SlamRobot::mapBuilderTimerCallback() {
    if (!map_initialized_) return;
    map_builder_.publishMap(this->now());
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
