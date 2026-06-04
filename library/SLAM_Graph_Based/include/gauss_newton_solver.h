/**
 * @file gauss_newton_solver.h
 * @brief Gauss-Newton pose graph optimizer for Graph-Based SLAM
 *
 * Reference: Grisetti, G., Kümmerle, R., Stachniss, C., & Burgard, W. (2010).
 *   "A Tutorial on Graph-Based SLAM." IEEE Intelligent Transportation Systems
 *   Magazine, 2(4), 31-43.
 *
 * ── Algorithm Overview ───────────────────────────────────────────────────────
 *
 *  The optimization goal is to find the set of poses x* that minimizes the
 *  total squared Mahalanobis distance of all constraint residuals:
 *
 *    x* = argmin_x  Σ_{<i,j>∈C}  e_ij(x_i, x_j)^T · Ω_ij · e_ij(x_i, x_j)
 *
 *  This is a nonlinear least-squares problem solved by Gauss-Newton iteration.
 *
 * ── One Gauss-Newton Iteration ──────────────────────────────────────────────
 *
 *  Step 1 — Linearize: for each edge (i, j):
 *    Compute residual:      e_ij  ∈ R^3           (see jacobian.h)
 *    Compute Jacobians:     A_ij, B_ij  ∈ R^{3×3}
 *
 *  Step 2 — Build linear system:
 *
 *    H_{ii} += A_ij^T · Ω_ij · A_ij      (3×3 block of H)
 *    H_{ij} += A_ij^T · Ω_ij · B_ij
 *    H_{ji} += B_ij^T · Ω_ij · A_ij
 *    H_{jj} += B_ij^T · Ω_ij · B_ij
 *
 *    b_i    += A_ij^T · Ω_ij · e_ij      (3-vector block of b)
 *    b_j    += B_ij^T · Ω_ij · e_ij
 *
 *  Step 3 — Fix gauge (anchor node 0 to avoid under-determination):
 *    Add large diagonal to H_{00}
 *
 *  Step 4 — Solve: H · Δξ = -b  (via Gaussian elimination)
 *
 *  Step 5 — Update: x_i ← x_i ⊕ Δξ_i
 *    In 2D:  x_i += Δξ_i[0],  y_i += Δξ_i[1],  θ_i += Δξ_i[2]
 *    followed by angle normalization on θ_i
 */

#ifndef SLAM_GRAPH_BASED_GAUSS_NEWTON_SOLVER_H
#define SLAM_GRAPH_BASED_GAUSS_NEWTON_SOLVER_H

#include "pose_graph.h"
#include "matrix.h"
#include "jacobian.h"

#include <vector>
#include <stdexcept>

namespace slam {

class GaussNewtonSolver {
public:
    /**
     * @brief Run a fixed number of Gauss-Newton iterations on the pose graph.
     *
     * Modifies the poses of all nodes in-place. Node 0 is anchored (gauge fix).
     *
     * @param graph  Pose graph to optimize (nodes modified in-place)
     * @param max_iterations  Number of Gauss-Newton steps (default: 10)
     */
    static void solve(PoseGraph2D& graph, int max_iterations = 10);
};

}  // namespace slam

#endif  // SLAM_GRAPH_BASED_GAUSS_NEWTON_SOLVER_H
