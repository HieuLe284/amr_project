#include "jacobian.h"

void slam::computeError(
    double xi, double yi, double ti,
    double xj, double yj, double tj,
    double zx, double zy, double zt,
    double e[3])
{
    // R_i^T = R(-θ_i)
    double Ri_T[2][2];
    makeRotation2D(-ti, Ri_T); // R(-θ) = R^T(θ)

    // R_ij^T = R(-θ_ij)
    double Rij_T[2][2];
    makeRotation2D(-zt, Rij_T);

    // dt = t_j - t_i
    double dtx = xj - xi;
    double dty = yj - yi;

    // R_i^T · dt
    double tmp[2];
    tmp[0] = Ri_T[0][0] * dtx + Ri_T[0][1] * dty;
    tmp[1] = Ri_T[1][0] * dtx + Ri_T[1][1] * dty;

    // R_i^T · dt - t_ij
    double diff[2] = { tmp[0] - zx, tmp[1] - zy };

    // R_ij^T · diff
    e[0] = Rij_T[0][0] * diff[0] + Rij_T[0][1] * diff[1];
    e[1] = Rij_T[1][0] * diff[0] + Rij_T[1][1] * diff[1];
    e[2] = normalizeAngle(tj - ti - zt);
}

void slam::computeJacobians(
    double xi, double yi, double ti,
    double xj, double yj, double /*tj*/,
    double /*zx*/, double /*zy*/, double zt,
    Mat3& A, Mat3& B)
{
    // R_i^T
    double Ri_T[2][2];
    makeRotation2D(-ti, Ri_T);

    // R_ij^T
    double Rij_T[2][2];
    makeRotation2D(-zt, Rij_T);

    // dR_i^T/dθ_i — Directly computed for correctness
    double dRi_T[2][2];
    makeDRotation2D(-ti, dRi_T); // dR^T/dθ = -(dR/dθ)^T evaluated at -θ
    // Note: dR(-θ)/d(-θ) = R'(-θ) so dR_i^T/dθ_i requires care.
    // Directly: R_i^T = [[cosθ, sinθ], [-sinθ, cosθ]]
    // d/dθ R_i^T = [[-sinθ, cosθ], [-cosθ, -sinθ]]
    dRi_T[0][0] = -std::sin(ti);  dRi_T[0][1] =  std::cos(ti);
    dRi_T[1][0] = -std::cos(ti);  dRi_T[1][1] = -std::sin(ti);

    // dt = t_j - t_i
    double dtx = xj - xi;
    double dty = yj - yi;

    // dR_i^T/dθ · (t_j - t_i) ∈ R^2
    double dRdt[2];
    dRdt[0] = dRi_T[0][0] * dtx + dRi_T[0][1] * dty;
    dRdt[1] = dRi_T[1][0] * dtx + dRi_T[1][1] * dty;

    // Compute: R_ij^T · [−R_i^T]  (2×2 block, upper-left of A)
    // −R_ij^T · R_i^T
    double neg_RijT_RiT[2][2];
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 2; ++c) {
            neg_RijT_RiT[r][c] = 0.0;
            for (int k = 0; k < 2; ++k)
                neg_RijT_RiT[r][c] += Rij_T[r][k] * (-Ri_T[k][c]);
        }

    // Compute: R_ij^T · dRdt  (2×1 block, third column of A, rows 0-1)
    double RijT_dRdt[2];
    RijT_dRdt[0] = Rij_T[0][0] * dRdt[0] + Rij_T[0][1] * dRdt[1];
    RijT_dRdt[1] = Rij_T[1][0] * dRdt[0] + Rij_T[1][1] * dRdt[1];

    // ── A_ij ─────────────────────────────────────────────────────────────
    //   [ -R_ij^T·R_i^T   |  R_ij^T·dR·dt ]
    //   [   0    0        |      -1        ]
    A(0,0) = neg_RijT_RiT[0][0];
    A(0,1) = neg_RijT_RiT[0][1];
    A(0,2) = RijT_dRdt[0];
    A(1,0) = neg_RijT_RiT[1][0];
    A(1,1) = neg_RijT_RiT[1][1];
    A(1,2) = RijT_dRdt[1];
    A(2,0) = 0.0;
    A(2,1) = 0.0;
    A(2,2) = -1.0;

    // ── B_ij ─────────────────────────────────────────────────────────────
    //   [  R_ij^T·R_i^T   |  0 ]
    //   [   0    0        |  1 ]
    double RijT_RiT[2][2];
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 2; ++c) {
            RijT_RiT[r][c] = 0.0;
            for (int k = 0; k < 2; ++k)
                RijT_RiT[r][c] += Rij_T[r][k] * Ri_T[k][c];
        }

    B(0,0) = RijT_RiT[0][0];
    B(0,1) = RijT_RiT[0][1];
    B(0,2) = 0.0;
    B(1,0) = RijT_RiT[1][0];
    B(1,1) = RijT_RiT[1][1];
    B(1,2) = 0.0;
    B(2,0) = 0.0;
    B(2,1) = 0.0;
    B(2,2) = 1.0;
}