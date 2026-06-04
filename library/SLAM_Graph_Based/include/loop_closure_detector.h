/**
 * @file loop_closure_detector.h
 * @brief Scan-correlation-based loop closure detection for Graph-Based SLAM
 *
 * Reference: Grisetti, G., Kümmerle, R., Stachniss, C., & Burgard, W. (2010).
 *   "A Tutorial on Graph-Based SLAM." IEEE Intelligent Transportation Systems
 *   Magazine, 2(4), 31-43.
 *
 * ── Loop Closure in Graph-Based SLAM ────────────────────────────────────────
 *
 *  A loop closure edge (i, j) is added to the graph when the robot revisits a
 *  previously explored region. The constraint z_ij is computed by matching the
 *  LiDAR scan at the new node j against the stored scan at candidate node i,
 *  yielding the relative transformation (δx, δy, δθ) with measurement noise.
 *
 *  This implementation uses a two-stage strategy:
 *
 *  Stage 1 — Proximity Filter:
 *    Only consider node pairs (i, j) where the Euclidean distance
 *    ‖t_j − t_i‖ < d_thresh AND the angular difference |θ_j − θ_i| < θ_thresh.
 *    This prunes the candidate set from O(N^2) to a small number.
 *
 *  Stage 2 — Scan Correlation Matching:
 *    Compute the normalized cross-correlation between range histograms:
 *
 *      C(z_i, z_j) = Σ_k [r_i(k) · r_j(k)] / (‖z_i‖ · ‖z_j‖)
 *
 *    where r_i(k) is the k-th range reading of scan i (NaN filled as 0).
 *    A match is declared if C > correlation_threshold.
 *
 *  Stage 3 — Relative Pose Estimation:
 *    For matched pairs, the relative measurement z_ij is computed from the
 *    current graph poses:
 *
 *      δt_ij = R_i^T · (t_j − t_i)
 *      δθ_ij = normalize(θ_j − θ_i)
 *
 *    This uses the graph poses as the initial alignment, consistent with the
 *    linearization point used in the Gauss-Newton solver.
 */

#ifndef SLAM_GRAPH_BASED_LOOP_CLOSURE_DETECTOR_H
#define SLAM_GRAPH_BASED_LOOP_CLOSURE_DETECTOR_H

#include "pose_graph.h"
#include "jacobian.h"  // for normalizeAngle

#include <vector>
#include <cmath>

namespace slam {

class LoopClosureDetector {
public:
    // ── Tunable parameters ────────────────────────────────────────────────
    double dist_threshold{2.0};          ///< max Euclidean proximity [m]
    double angle_threshold{1.2};         ///< max angle proximity [rad]
    double correlation_threshold{0.80};  ///< min scan correlation score [0,1]
    int    min_node_gap{10};             ///< min index gap to avoid self-match

    // Information matrix weights for loop-closure edges
    // (higher than odometry = more confidence in scan matching)
    double omega_xy{200.0};
    double omega_theta{400.0};

    /**
     * @brief Detect loop closures for a newly added node.
     *
     * Searches through all nodes older than min_node_gap and applies the
     * two-stage proximity + correlation filter to find matching candidates.
     *
     * @param graph     Pose graph (must have nodes with scan data populated)
     * @param new_idx   Index of the newly added node to match against
     * @return          Index of the best matching older node, or -1 if none
     */
    int detect(PoseGraph2D& graph, int new_idx);

private:
    /**
     * @brief Normalized cross-correlation between two range scan vectors.
     *
     * C = Σ r_i · r_j / (‖r_i‖ · ‖r_j‖)
     *
     * Returns value in [0, 1]; NaN/inf ranges are clamped to 0.
     */
    static double scanCorrelation(const std::vector<double>& a,
                                   const std::vector<double>& b);
};

}  // namespace slam

#endif  // SLAM_GRAPH_BASED_LOOP_CLOSURE_DETECTOR_H
