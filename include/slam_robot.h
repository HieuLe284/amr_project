#ifndef SLAM_ROBOT_H
#define SLAM_ROBOT_H

// ── ROS2 core ────────────────────────────────────────────────────────────────
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

// ── Math utilities ────────────────────────────────────────────────────────────
#include "library/common/math_utils.h"

// ── Standard library ─────────────────────────────────────────────────────────
#include <atomic>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

// ── Graph-Based SLAM Library (lib/SLAM_Graph_Based) ──────────────────────────
#include "library/SLAM_Graph_Based/include/slam_graph.h"
#include "library/SLAM_Graph_Based/include/map_builder.h"

using std::placeholders::_1;

// ═════════════════════════════════════════════════════════════════════════════
//  SlamRobot — ROS2 Node thực hiện Graph-Based SLAM
//  Kiến trúc hệ thống:
//    • scanCallback()
//        - Nhận dữ liệu LiDAR
//        - Lưu scan vào bộ nhớ đệm (cache)
//        - Cập nhật / xuất bản bản đồ
//    • slamTimerCallback()
//        - Lấy pose robot từ TF
//        - Gọi graphSLAMcall()
//        - Xuất bản dữ liệu trực quan hóa Pose Graph
//    • graphSLAMcall()
//        - Hàm SLAM duy nhất của toàn hệ thống:
//            1) addOdometryNode()  (Front-End)
//            2) addLoopClosures() (Front-End)
//            3) optimizeIfNeeded() (Back-End Gauss-Newton)
//            4) Rebuild Map nếu đồ thị được tối ưu
// ═════════════════════════════════════════════════════════════════════════════
class SlamRobot : public rclcpp::Node {
public:
    SlamRobot();

private:
    /**
     * @brief Helper chuyển tf2::Quaternion sang góc yaw [rad]
     * @param q Tham chiếu tf2::Quaternion
     * @return Góc yaw [rad]
     */
    static inline double getYaw(const tf2::Quaternion& q) {
        return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                         1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }

    /**
     * @brief Phát TF từ map → odom. Được sử dụng để đồng bộ hệ tọa độ bản đồ (map)
     * và hệ tọa độ odometry (odom).
     */
    void broadcastMapOdomTF(rclcpp::Time now);

    /**
     * Luồng xử lý:
     *      TF
     *       ↓
     *  graphSLAMcall()
     *       ↓
     *  Publish Pose Graph
     *       ↓
     *  Publish Visualization
     *       ↓
     *     Rviz
     */
    void slamTimerCallback();

    /**
     * @brief Cập nhật phép biến đổi map → odom.
     * @param map_x,map_y,map_theta Pose trong hệ map.
     * @param odom_x,odom_y,odom_theta Pose trong hệ odom.
     */
    void updateMapOdom(double map_x, double map_y, double map_theta,
                       double odom_x, double odom_y, double odom_theta);

    /**
     * @brief Callback nhận dữ liệu LiDAR.
     * Chức năng:
     *   • Lưu LaserScan vào cache.
     *   • Cung cấp dữ liệu cho: graphSLAMcall()
     *   • Cập nhật MapBuilder liên tục.
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // ── Map publish timer ─────────────────────────────────────────────────
    void mapBuilderTimerCallback();

    /**
     * @brief Hàm xử lý SLAM duy nhất của hệ thống. 
     * Thực hiện đầy đủ một vòng Graph-Based SLAM. 
     * ────────────────────────────────────────────────────────────
     * Bước 1 — Front-End (Odometry) 
     *   Tính chuyển động tương đối: 
     *       δ = (δx, δy, δθ) từ node trước đó. 
     *   Thêm:
     *       • Node mới
     *       • Cạnh Odometry vào Pose Graph.
     * ────────────────────────────────────────────────────────────
     * Bước 2 — Front-End (Loop Closure)
     *   Tìm kiếm các node cũ.
     *   So sánh dữ liệu LiDAR bằng: Scan Correlation Nếu độ tương 
     *   đồng vượt ngưỡng: Thêm cạnh Loop Closure với trọng số thông 
     *   tin lớn hơn.
     * ────────────────────────────────────────────────────────────
     * Bước 3 — Back-End (Gauss-Newton)
     *   Nếu vừa phát hiện Loop Closure:
     *       x* = argmin Σ e_ijᵀ Ω_ij e_ij
     *   Xây dựng:
     *       HΔξ = -b
     *   Giải bằng: Gaussian Elimination Sau đó cập nhật toàn bộ pose.
     * ────────────────────────────────────────────────────────────
     * Bước 4 — Tái tạo bản đồ
     *   Nếu đồ thị vừa được tối ưu: clearMap()
     *   Sau đó: updateMapFromNode() cho toàn bộ node trong graph.

     * Bước 5 — Cập nhật map→odom TF
     * @param x,y,theta Pose hiện tại lấy từ: odom → base_link
     * * @return -1 nếu không có node mới, 0 nếu có node mới, 1 nếu có node mới + optimized
     */
    int graphSLAMcall(double x, double y, double theta);

    // TF map → odom (hiệu chỉnh SLAM)
    double map_odom_x = 0.0, map_odom_y = 0.0, map_odom_theta = 0.0; // map→odom TF (identity)

    // ── Publishers ────────────────────────────────────────────────────────
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr      pub_map_; // Occupancy Grid Map
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr     pub_graph_nodes_; // Pose Graph nodes
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_graph_edges_; // Pose Graph edges
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             pub_loop_closure_event_; // Thông báo phát hiện Loop Closure
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr       pub_scan_visualization_; // LaserScan dùng cho trực quan hóa

    // ── Subscribers ───────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_; // Nhận dữ liệu LiDAR

    // ── Timers ────────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr timer_; // 5 Hz SLAM timer (no viz!)
    rclcpp::TimerBase::SharedPtr tf_broadcast_timer_; // 50 Hz TF broadcast (decoupled from SLAM)
    rclcpp::TimerBase::SharedPtr map_timer_; // 2 Hz map publish timer
    rclcpp::TimerBase::SharedPtr graph_viz_timer_; // 0.5 Hz graph visualization

    // ── Graph viz dirty flag ──────────────────────────────────────────────
    std::atomic<bool> graph_viz_dirty_{true};
    bool graph_has_changed_{false}; // set by graphSLAMcall, read+cleared by viz timer
    void graphVizTimerCallback();

    // ── TF2 ──────────────────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::Buffer>              tf_buffer_;       // Bộ đệm TF
    std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;     // Nhận TF
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // Phát TF

    // ── Callback Groups ───────────────────────────────────────────────────
    rclcpp::CallbackGroup::SharedPtr callback_group_lidar_; // Nhóm callback LiDAR
    rclcpp::CallbackGroup::SharedPtr callback_group_slam_; // Nhóm callback SLAM

    // ── Graph-Based SLAM ──────────────────────────────────────────────────
    slam::SlamGraph  slam_graph_; // Full SLAM pipeline 
    slam::MapBuilder map_builder_; // Log-Odds occupancy grid

    // ── Dữ liệu LiDAR được lưu tạm ────────────────────────────────────────
    std::vector<double> cached_scan_ranges_; // Chia sẻ giữa scanCallback & graphSLAMcall
    double cached_scan_angle_min_{0.0}; // Góc bắt đầu của LaserScan
    double cached_scan_angle_increment_{0.0}; // Độ phân giải góc

    // ── Map state ─────────────────────────────────────────────────────────
    bool map_initialized_{false}; // true = khởi tạo

    // ── Cached OccupancyGrid (chỉ build 1 lần, dùng cho cả frontier + A* + publish) ──
    std::mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid cached_occ_grid_; // snapshot của occupancy grid
    bool cached_grid_valid_{false};
    void refreshCachedGrid(); // build fresh + swap

};

#endif  // SLAM_ROBOT_H
