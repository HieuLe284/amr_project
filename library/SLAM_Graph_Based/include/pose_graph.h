/**
 * @file pose_graph.h
 * @brief Pose-Graph data structures for Graph-Based SLAM
 *
 * Reference: Grisetti, G., Kümmerle, R., Stachniss, C., & Burgard, W. (2010).
 *   "A Tutorial on Graph-Based SLAM." IEEE Intelligent Transportation Systems
 *   Magazine, 2(4), 31-43.
 *
 * ── Data Model ───────────────────────────────────────────────────────────────
 *
 *   The pose graph G = (V, E) consists of:
 *
 *   • Vertices (Nodes): each node x_i = (x_i, y_i, θ_i)^T represents an
 *     estimated robot pose in the global reference frame, also storing the
 *     LiDAR scan taken at that pose.
 *
 *   • Edges: each edge (i, j, z_ij, Ω_ij) encodes a spatial constraint
 *     — the relative transformation z_ij measured between pose i and pose j,
 *     weighted by the information matrix Ω_ij = Σ_ij^{-1} (inverse covariance).
 *
 *   Odometry edges: consecutive (i → i+1), low angular info (tend to drift).
 *   Loop-closure edges: non-consecutive, from scan-matching; high weight.
 */

#ifndef SLAM_GRAPH_BASED_POSE_GRAPH_H
#define SLAM_GRAPH_BASED_POSE_GRAPH_H

#include <vector>
#include <cmath>

namespace slam {

// ═══════════════════════════════════════════════════════════════════════════
//  Node2D — vertex of the pose graph
//
//  Represents robot pose x_i = (x, y, θ)^T in the global map frame.
//  Also stores the LiDAR scan taken at this pose for loop-closure matching.
// ═══════════════════════════════════════════════════════════════════════════
struct Node2D {
    double x{0.0};      ///< Global x-position  [m]
    double y{0.0};      ///< Global y-position  [m]
    double theta{0.0};  ///< Global heading θ   [rad]

    /// LiDAR scan data recorded at this pose (for loop closure)
    std::vector<double> scan_ranges;
    double scan_angle_min{0.0};
    double scan_angle_increment{0.0};

    Node2D() = default;
    Node2D(double x_, double y_, double t_) : x(x_), y(y_), theta(t_) {}
};

// ═══════════════════════════════════════════════════════════════════════════
//  Edge2D — constraint between two nodes in the pose graph
//
//  Encodes the relative measurement z_ij = (δx, δy, δθ)^T from pose i
//  to pose j, along with the 3×3 information matrix Ω_ij.
//
//  Information matrix Ω_ij = Σ_ij^{-1}:
//    - Diagonal entries represent confidence in each DoF.
//    - Odometry edges: moderate confidence (noise model of wheel odometry).
//    - Loop-closure edges: higher confidence (scan-matching is more precise).
//
//  Storage: Ω is stored as a symmetric 3×3 matrix (9 values).
// ═══════════════════════════════════════════════════════════════════════════
struct Edge2D {
    int from{-1};       ///< Index of source node i
    int to{-1};         ///< Index of target node j
    bool is_loop{false};///< True if this is a loop-closure constraint

    /// Relative measurement z_ij = (δx, δy, δθ)^T
    double z_x{0.0};
    double z_y{0.0};
    double z_theta{0.0};

    /// Information matrix Ω_ij [3×3] — inverse covariance of the constraint
    ///   Ω = diag(ω_x, ω_y, ω_θ) for diagonal case
    double omega[3][3]{};

    Edge2D();

    /// Convenience: set diagonal information matrix
    void setOmegaDiagonal(double wx, double wy, double wt);
};

// ═══════════════════════════════════════════════════════════════════════════
//  PoseGraph2D — the complete graph G = (V, E)
// ═══════════════════════════════════════════════════════════════════════════
struct PoseGraph2D {
    std::vector<Node2D> nodes;  ///< V = {x_0, x_1, ..., x_{N-1}}
    std::vector<Edge2D> edges;  ///< E = {e_ij | (i,j) ∈ constraints}

    int numNodes() const;
    int numEdges() const;

    /// Add an odometry edge i → j
    void addEdge(int i, int j,
                 double zx, double zy, double zt,
                 double wx, double wy, double wt,
                 bool loop = false);
};

}  // namespace slam

#endif  // SLAM_GRAPH_BASED_POSE_GRAPH_H
