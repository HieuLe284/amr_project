/**
 * @file jacobian.h
 * @brief Analytical Jacobians for 2D Pose-Pose constraints in Graph-Based SLAM
 *
 * Reference: Grisetti, G., Kümmerle, R., Stachniss, C., & Burgard, W. (2010).
 *   "A Tutorial on Graph-Based SLAM." IEEE Intelligent Transportation Systems
 *   Magazine, 2(4), 31-43.
 *
 * ── Pose Representation (2D SE(2)) ──────────────────────────────────────────
 *
 *   Robot pose at node i:   x_i = (x_i, y_i, θ_i)^T
 *
 *   Homogeneous transformation matrix:
 *
 *       ⎡ cos θ_i  -sin θ_i  x_i ⎤
 *   X_i = ⎢ sin θ_i   cos θ_i  y_i ⎥
 *       ⎣    0          0      1  ⎦
 *
 * ── Edge Error Function ──────────────────────────────────────────────────────
 *
 *   Given a constraint between node i and node j with relative measurement
 *   z_ij = (t_ij^T, θ_ij)^T (i.e., the relative pose of j seen from i):
 *
 *   e_ij(x_i, x_j) = t2v( Z_ij^{-1} · X_i^{-1} · X_j )
 *
 *   Expanded form:
 *
 *   e_ij = ⎡ R_ij^T · (R_i^T · (t_j - t_i) - t_ij) ⎤
 *          ⎣ normalize(θ_j - θ_i - θ_ij)             ⎦
 *
 * ── Jacobians ────────────────────────────────────────────────────────────────
 *
 *   The Jacobian J_ij = [A_ij | B_ij] where:
 *
 *     A_ij = ∂e_ij / ∂x_i  (3×3)
 *     B_ij = ∂e_ij / ∂x_j  (3×3)
 *
 *   Let  dR_i = ∂R_i^T/∂θ_i = ⎡ -sinθ_i  cosθ_i ⎤
 *                               ⎣ -cosθ_i -sinθ_i ⎦
 *
 *   A_ij = ⎡ -R_ij^T · R_i^T     R_ij^T · dR_i · (t_j - t_i) ⎤
 *          ⎣  0    0              -1                             ⎦
 *
 *   B_ij = ⎡  R_ij^T · R_i^T     0 ⎤
 *          ⎣  0    0              1 ⎦
 */

#ifndef SLAM_GRAPH_BASED_JACOBIAN_H
#define SLAM_GRAPH_BASED_JACOBIAN_H

#include "matrix.h"
#include <cmath>

namespace slam {

// ── angle normalization ──────────────────────────────────────────────────────
inline double normalizeAngle(double a){
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ═══════════════════════════════════════════════════════════════════════════
//  2D Rotation matrix R(θ) as 2×2 packed into upper-left of Mat3
// ═══════════════════════════════════════════════════════════════════════════
inline void makeRotation2D(double theta, double R[2][2]) {
    R[0][0] =  std::cos(theta);  R[0][1] = -std::sin(theta);
    R[1][0] =  std::sin(theta);  R[1][1] =  std::cos(theta);
}

// ── dR^T/dθ  (derivative of R^T w.r.t. θ) ──────────────────────────────────
// dR_i^T / dθ_i = ⎡ -sinθ  cosθ ⎤
//                  ⎣ -cosθ -sinθ ⎦
inline void makeDRotation2D(double theta, double dR[2][2]) {
    dR[0][0] = -std::sin(theta);  dR[0][1] =  std::cos(theta);
    dR[1][0] = -std::cos(theta);  dR[1][1] = -std::sin(theta);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Edge error  e_ij(x_i, x_j)  ∈ R^3
//
//  Inputs:
//    x{i,j}        — pose vectors (x, y, θ)
//    z_{x,y,theta} — relative measurement (translation + rotation)
//  Output:
//    e[3]          — residual vector
// ═══════════════════════════════════════════════════════════════════════════
void computeError(
    double xi, double yi, double ti,        // pose i
    double xj, double yj, double tj,        // pose j
    double zx, double zy, double zt,        // measurement z_ij
    double e[3]);

// ═══════════════════════════════════════════════════════════════════════════
//  Analytical Jacobians  A_ij (3×3) and B_ij (3×3)
//
//  A_ij = ∂e_ij/∂x_i ,   B_ij = ∂e_ij/∂x_j
//
//  See Section III-C of Grisetti et al. (2010) for derivation.
// ═══════════════════════════════════════════════════════════════════════════
void computeJacobians(
    double xi, double yi, double ti,        // pose i
    double xj, double yj, double tj,        // pose j
    double zx, double zy, double zt,        // measurement z_ij
    Mat3& A, Mat3& B);

}  // namespace slam

#endif  // SLAM_GRAPH_BASED_JACOBIAN_H
