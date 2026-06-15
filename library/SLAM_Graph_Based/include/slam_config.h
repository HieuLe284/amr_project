#ifndef SLAM_CONFIG_H
#define SLAM_CONFIG_H

/**
 * @struct SlamConfig
 * @brief Tham số cấu hình cho thuật toán Graph_SLAM
 */
struct SlamConfig {
    // ── Ngưỡng di chuyển tối thiểu trước khi tạo node mới ─────────────────
    double min_travel_dist;     // Khoảng cách tối thiểu [m]
    double min_travel_angle;    // Góc quay tối thiểu [rad]

    // ── Số vòng lặp Gauss-Newton mỗi lần tối ưu ───────────────────────────
    double gn_iterations;

    // ── Trọng số ma trận thông tin của cạnh Odometry ──────────────────────
    double odom_omega_xy;       // Trọng số theo phương x và y
    double odom_omega_theta;    // Trọng số theo góc θ

    // ── Các tham số có thể điều chỉnh ( Tunable parameters ) ─────────────────
    double dist_threshold;          // Ngưỡng khoảng cách Euclid lớn nhất giữa hai node [m]
    double angle_threshold;         // Ngưỡng chênh lệch góc lớn nhất giữa hai node [rad]
    double correlation_threshold;  // Ngưỡng tương quan scan tối thiểu [0,1]
    int    min_node_gap;             // Khoảng cách chỉ số node tối thiểu nhằm tránh so khớp với các node quá gần về thời gian
    
    // ── Trọng số ma trận thông tin cho cạnh Loop Closure ─────────────────────
    // Giá trị lớn hơn cạnh odometry → thể hiện mức độ tin cậy cao hơn đối với ràng buộc đóng vòng lặp
    double omega_xy{200.0};         // Trọng số loop closure (x, y) 
    double omega_theta{400.0};      // Trọng số loop closure (θ)

    SlamConfig(): 
        min_travel_dist(0.3),       
        min_travel_angle(0.15),        
        gn_iterations(10),                
        odom_omega_xy(50.0),                
        odom_omega_theta(80.0),       
        dist_threshold(2.0),
        angle_threshold(1.2),
        correlation_threshold(0.80),
        min_node_gap(10),
        omega_xy(200.0),
        omega_theta(400.0) {}  
};

#endif  // SLAM_CONFIG_H
