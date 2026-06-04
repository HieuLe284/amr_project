#include "loop_closure_detector.h"

int slam::LoopClosureDetector::detect(PoseGraph2D& graph, int new_idx) {
    const int N = graph.numNodes();
    if (new_idx < min_node_gap) return -1;

    const Node2D& nj = graph.nodes[new_idx];
    if (nj.scan_ranges.empty()) return -1;

    int    best_id    = -1;
    double best_score = correlation_threshold;

    for (int i = 0; i < new_idx - min_node_gap; ++i) {
        const Node2D& ni = graph.nodes[i];
        if (ni.scan_ranges.empty()) continue;

        // ── Stage 1: Proximity filter ─────────────────────────────────
        double dx = nj.x - ni.x;
        double dy = nj.y - ni.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist > dist_threshold) continue;

        double dtheta = std::fabs(normalizeAngle(nj.theta - ni.theta));
        if (dtheta > angle_threshold) continue;

        // ── Stage 2: Scan correlation ─────────────────────────────────
        double score = scanCorrelation(ni.scan_ranges, nj.scan_ranges);
        if (score > best_score) {
            best_score = score;
            best_id    = i;
        }
    }

    if (best_id < 0) return -1;

    // ── Stage 3: Compute relative pose z_ij from current graph poses ──
    const Node2D& ni = graph.nodes[best_id];

    // R_i^T = rotation matrix at −θ_i
    double ci = std::cos(ni.theta);
    double si = std::sin(ni.theta);
    double dtx = nj.x - ni.x;
    double dty = nj.y - ni.y;

    // δt_ij = R_i^T · (t_j − t_i)
    double zx =  ci * dtx + si * dty;
    double zy = -si * dtx + ci * dty;
    double zt =  normalizeAngle(nj.theta - ni.theta);

    // Add loop-closure edge j → best_id
    graph.addEdge(best_id, new_idx,
                  zx, zy, zt,
                  omega_xy, omega_xy, omega_theta,
                  /*loop=*/true);

    return best_id;
}

double slam::LoopClosureDetector::scanCorrelation(const std::vector<double>& a,
                                             const std::vector<double>& b)
{
    size_t n = std::min(a.size(), b.size());
    if (n == 0) return 0.0;

    double dot = 0.0, na = 0.0, nb = 0.0;
    for (size_t k = 0; k < n; ++k) {
        double ra = (std::isfinite(a[k]) && a[k] > 0.0) ? a[k] : 0.0;
        double rb = (std::isfinite(b[k]) && b[k] > 0.0) ? b[k] : 0.0;
        dot += ra * rb;
        na  += ra * ra;
        nb  += rb * rb;
    }
    double denom = std::sqrt(na) * std::sqrt(nb);
    return (denom > 1e-9) ? (dot / denom) : 0.0;
}
