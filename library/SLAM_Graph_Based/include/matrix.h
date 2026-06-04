/**
 * @file matrix.h
 * @brief Dense matrix arithmetic for Graph-Based SLAM (Grisetti et al., 2010)
 *
 * Implements a general-purpose dense NxN matrix used for constructing the
 * global information matrix H (Hessian approximation) and the coefficient
 * vector b in the Gauss-Newton pose-graph optimizer.
 *
 * The linear system to solve at each iteration is:
 *
 *   H · Δξ = -b
 *
 * where H ∈ R^{3N×3N} and b ∈ R^{3N}.
 */

#ifndef SLAM_GRAPH_BASED_MATRIX_H
#define SLAM_GRAPH_BASED_MATRIX_H

#include <vector>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace slam {

// ═══════════════════════════════════════════════════════════════════════════
//  3×3 fixed matrix — used for Jacobian blocks A_ij, B_ij, Ω_ij
// ═══════════════════════════════════════════════════════════════════════════
struct Mat3 {
    double d[3][3];

    Mat3();

    explicit Mat3(double diag);

    /// Element access
    double& operator()(int r, int c);
    double  operator()(int r, int c) const;

    /// Matrix multiply C = A * B  (3×3)
    static Mat3 mul(const Mat3& A, const Mat3& B);

    /// Matrix transpose
    Mat3 T() const;

    /// A^T * Ω * B  (used in H assembly)
    static Mat3 AtOmegaB(const Mat3& A, const Mat3& Omega, const Mat3& B);

    /// Multiply 3×3 matrix by 3-vector: y = M * x
    static void vecMul(const Mat3& M, const double x[3], double y[3]);

    /// Add in-place: this += rhs
    Mat3& operator+=(const Mat3& rhs);
};

// ═══════════════════════════════════════════════════════════════════════════
//  Dense NxN matrix — used for the global Hessian H ∈ R^{3N×3N}
//  and coefficient vector b ∈ R^{3N}
// ═══════════════════════════════════════════════════════════════════════════
class MatrixX {
public:
    int rows, cols;
    std::vector<double> data;

    MatrixX();
    MatrixX(int r, int c, double fill = 0.0);

    double& at(int r, int c);
    double  at(int r, int c) const;

    /// Augmented 3×3 block at (bi,bj): H_bi_bj += M
    void addBlock(int bi, int bj, const Mat3& M);

    void setZero();
};

// ═══════════════════════════════════════════════════════════════════════════
//  Gaussian Elimination with partial pivoting
//
//  Solves the linear system  H · Δξ = -b
//  Returns Δξ as a flat vector of length N.
//  Throws std::runtime_error if matrix is (near-)singular.
// ═══════════════════════════════════════════════════════════════════════════
std::vector<double> solveLinearSystem(MatrixX H, std::vector<double> rhs);

}  // namespace slam

#endif  // SLAM_GRAPH_BASED_MATRIX_H
