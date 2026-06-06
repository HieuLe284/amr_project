#ifndef SLAM_ROBOT_H
#define SLAM_ROBOT_H

// ── ROS2 core ────────────────────────────────────────────────────────────────
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
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

// ── Standard library ─────────────────────────────────────────────────────────
#include <atomic>
#include <cmath>
#include <string>
#include <vector>

// ── Graph-Based SLAM Library (lib/SLAM_Graph_Based) ──────────────────────────
//   Implements the algorithm from:
//   Grisetti, G., Kümmerle, R., Stachniss, C., & Burgard, W. (2010).
//   "A Tutorial on Graph-Based SLAM."
//   IEEE Intelligent Transportation Systems Magazine, 2(4), 31–43.
#include "library/SLAM_Graph_Based/include/slam_graph.h"
#include "library/SLAM_Graph_Based/include/map_builder.h"

using std::placeholders::_1;

// ═════════════════════════════════════════════════════════════════════════════
//  SlamRobot — ROS2 Node for Graph-Based SLAM
//
//  Architecture:
//    • scanCallback      — Receives LiDAR, caches scan data, publishes map
//    • slamTimerCallback — Gets TF pose, calls graphSLAMcall(), publishes viz
//    • graphSLAMcall     — THE only SLAM logic function:
//                           1) addOdometryNode  (front-end)
//                           2) addLoopClosures  (front-end)
//                           3) optimizeIfNeeded (back-end Gauss-Newton)
//                           4) rebuild map if optimized
// ═════════════════════════════════════════════════════════════════════════════
class SlamRobot : public rclcpp::Node {
public:
    SlamRobot();

private:
    // ── angle normalization ──────────────────────────────────────────────────────
    inline double normalizeAngle(double a){
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    void broadcastMapOdomTF(rclcpp::Time now);

    // ── SLAM timer: get TF pose → graphSLAMcall → publish graph viz ──────
    void slamTimerCallback();

    void updateMapOdom(double map_x, double map_y, double map_theta,
                       double odom_x, double odom_y, double odom_theta);

    // ── Scan callback: cache LiDAR data, feed MapBuilder continuously ─────
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // ── Map publish timer ─────────────────────────────────────────────────
    void mapBuilderTimerCallback();

    /**
     * @brief  THE single SLAM processing function.
     *
     * Runs the complete Graph-Based SLAM pipeline for one step:
     *
     *   Step 1 — Front-End (add odometry node):
     *     Computes body-frame delta δ = (δx, δy, δθ) from previous node
     *     and adds a new node + odometry edge to the pose graph.
     *
     *   Step 2 — Front-End (loop closure detection):
     *     Searches older nodes for scan correlation match > threshold.
     *     If matched: adds loop-closure edge with higher information weight.
     *
     *   Step 3 — Back-End (Gauss-Newton optimization):
     *     If a loop was just detected:
     *       x* = argmin Σ_{<i,j>} e_ij^T · Ω_ij · e_ij
     *       Solves H·Δξ = -b via Gaussian elimination, updates all poses.
     *
     *   Step 4 — Map rebuild:
     *     If optimized: clear grid, re-ray-cast all nodes' scans.
     *
     * @param x, y, theta  Current pose from TF (map → base_link)
     */
    void graphSLAMcall(double x, double y, double theta);

    double map_odom_x = 0.0, map_odom_y = 0.0, map_odom_theta = 0.0; // map→odom TF (identity)

    // ── Publishers ────────────────────────────────────────────────────────
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         pub_cmd_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr      pub_map_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr     pub_graph_nodes_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_graph_edges_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             pub_loop_closure_event_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr       pub_scan_visualization_;

    // ── Subscribers ───────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

    // ── Timers ────────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr timer_;      ///< 5 Hz SLAM + viz timer
    rclcpp::TimerBase::SharedPtr map_timer_;  ///< 5 Hz map publish timer

    // ── TF2 ──────────────────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::Buffer>              tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ── Callback Groups ───────────────────────────────────────────────────
    rclcpp::CallbackGroup::SharedPtr callback_group_lidar_;
    rclcpp::CallbackGroup::SharedPtr callback_group_slam_;

    // ── Graph-Based SLAM ──────────────────────────────────────────────────
    slam::SlamGraph  slam_graph_;   ///< Full SLAM pipeline (Grisetti et al.)
    slam::MapBuilder map_builder_;  ///< Log-Odds occupancy grid

    // ── Cached LiDAR data (shared between scanCallback & graphSLAMcall) ──
    std::vector<double> cached_scan_ranges_;
    double cached_scan_angle_min_{0.0};
    double cached_scan_angle_increment_{0.0};

    // ── Map state ─────────────────────────────────────────────────────────
    bool map_initialized_{false};
};

#endif  // SLAM_ROBOT_H
