/**
 * @file slam_graph.h
 * @brief SlamGraph — Coordinator for Graph-Based SLAM pipeline
 *
 * Reference: Grisetti, G., Kümmerle, R., Stachniss, C., & Burgard, W. (2010).
 *   "A Tutorial on Graph-Based SLAM." IEEE Intelligent Transportation Systems
 *   Magazine, 2(4), 31-43.
 *
 * ── Pipeline Overview ────────────────────────────────────────────────────────
 *
 *   At each time step t:
 *
 *   1. FRONT-END — addOdometryNode(x, y, θ, scan):
 *      • Compute body-frame delta from previous node:
 *          δx = R_{i-1}^T · (t_i − t_{i-1})
 *          δθ = normalize(θ_i − θ_{i-1})
 *      • Add new node x_i to graph
 *      • Add odometry edge (i-1 → i) with measurement δx, diagonal Ω_odom
 *
 *   2. LOOP CLOSURE DETECTION — addLoopClosures(new_idx):
 *      • Search older nodes for scan correlation match
 *      • If match found: add loop-closure edge with higher Ω_loop
 *      • Count total loop closures
 *
 *   3. BACK-END OPTIMIZATION — optimizeIfNeeded():
 *      • If a new loop closure was found in this call:
 *          Run Gauss-Newton iterations: x* = argmin Σ e_ij^T Ω_ij e_ij
 *      • Returns true if optimization ran (signals caller to rebuild map)
 *
 *   4. MAP REBUILD — performed by caller (slam_robot.cpp) using getNodes()
 *      for each optimized node, call mapBuilder_.updateFromRanges()
 *
 * ── Odometry Noise Model ─────────────────────────────────────────────────────
 *
 *   Information matrix for odometry edges (diagonal approximation):
 *     Ω_odom = diag(ω_x, ω_y, ω_θ)
 *
 *   Information matrix for loop-closure edges:
 *     Ω_loop = diag(ω_x_loop, ω_y_loop, ω_θ_loop)  (higher values)
 */

#ifndef SLAM_GRAPH_BASED_SLAM_GRAPH_H
#define SLAM_GRAPH_BASED_SLAM_GRAPH_H

#include "pose_graph.h"
#include "gauss_newton_solver.h"
#include "loop_closure_detector.h"
#include "map_builder.h"
#include "jacobian.h"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <vector>

namespace slam {

class SlamGraph {
public:
    // ── Public state (accessed by slam_robot.cpp for visualization) ───────
    PoseGraph2D          pose_graph;         ///< G = (V, E)
    int                  loop_closure_count_{0};

    // ── Odometry edge information weights ─────────────────────────────────
    double odom_omega_xy{50.0};
    double odom_omega_theta{80.0};

    // ── Gauss-Newton iterations per optimization ───────────────────────────
    int gn_iterations{10};

    // ── Minimum travel distance before adding new node ────────────────────
    double min_travel_dist{0.30};    ///< [m]
    double min_travel_angle{0.15};   ///< [rad]

    SlamGraph() = default;

    /**
     * @brief Initialize the graph with an anchor node at (0, 0, 0).
     * Must be called once before processing any sensor data.
     */
    void init();

    /**
     * @brief Set the map builder (for map rebuild after optimization).
     * @param mb  Reference to a MapBuilder instance (owned by slam_robot)
     */
    void setMapBuilder(MapBuilder* mb);

    /**
     * @brief [Front-End] Add an odometry node to the pose graph.
     *
     * Only adds a new node if the robot has moved sufficiently (min_travel_dist
     * or min_travel_angle) since the last node, to keep the graph manageable.
     *
     * Odometry edge measurement z_{i-1,i}:
     *   δt = R_{i-1}^T · (t_i − t_{i-1})     ← body-frame translation delta
     *   δθ = normalize(θ_i − θ_{i-1})
     *
     * @param x, y, theta  Current robot pose (from TF: map → base_link)
     * @param ranges       Current LiDAR ranges
     * @param angle_min, angle_inc  Scan geometry
     * @return Index of the newly added node, or -1 if no node was added
     */
    int addOdometryNode(double x, double y, double theta,
                        const std::vector<double>& ranges,
                        double angle_min, double angle_inc);
    /**
     * @brief [Front-End] Detect loop closures for the most recently added node.
     *
     * @param new_idx  Index returned by the last successful addOdometryNode()
     * @return         Index of matched older node, or -1 if no closure found
     */
    int addLoopClosures(int new_idx);

    /**
     * @brief [Back-End] Run Gauss-Newton optimization if a loop closure
     *        was detected in the current step.
     *
     * x* = argmin_x  Σ_{<i,j>∈C}  e_ij^T Ω_ij e_ij
     *
     * @return true if optimization ran (caller should rebuild the map)
     */
    bool optimizeIfNeeded();

    // ── Map Builder passthrough ────────────────────────────────────────────
    void clearMap();

    void updateMapFromNode(int node_idx);

    // ── Tuning proxies ────────────────────────────────────────────────────
    LoopClosureDetector& loopDetector();

private:
    LoopClosureDetector  loop_detector_;
    MapBuilder*          map_builder_{nullptr};
    bool                 new_loop_this_step_{false};
};

}  // namespace slam

#endif  // SLAM_GRAPH_BASED_SLAM_GRAPH_H
