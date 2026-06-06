/**
 * @file map_builder.h
 * @brief Occupancy grid mapping with Log-Odds update for Graph-Based SLAM
 *
 * Implements a probabilistic occupancy grid using the Log-Odds representation:
 *
 *   l_t(x) = l_{t-1}(x) + log[ p(x|z_t) / (1 - p(x|z_t)) ]
 *                        − log[ p_0 / (1 - p_0) ]
 *
 * where p_0 = 0.5 is the uniform prior (log-odds = 0).
 *
 * Cell states (ROS OccupancyGrid convention):
 *   -1   → unknown  (|l| < free_thresh)
 *    0   → free     (l < −free_thresh)
 *  100   → occupied (l > occ_thresh)
 *
 * Each LiDAR beam is ray-cast using Bresenham's line algorithm:
 *   • Cells along the ray up to the hit are marked free  (l += l_free)
 *   • The cell at the endpoint is marked occupied        (l += l_occ)
 */

#ifndef SLAM_GRAPH_BASED_MAP_BUILDER_H
#define SLAM_GRAPH_BASED_MAP_BUILDER_H

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <algorithm>
#include <cmath>

namespace slam {

class MapBuilder {
public:
    MapBuilder() = default;
    /**
     * @param resolution  Grid resolution [m/cell]
     * @param width       Grid width  [cells]
     * @param height      Grid height [cells]
     * @param origin_x    World x-coordinate of grid cell (0,0) [m]
     * @param origin_y    World y-coordinate of grid cell (0,0) [m]
     */
    MapBuilder(double resolution, int width, int height,
               double origin_x, double origin_y);

    void setPublisher(rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub);

    /// Clear all log-odds to 0 (unknown)
    void clearMap();

    /**
     * @brief Update grid from a single LiDAR scan at a known pose.
     *
     * For each valid range reading r at angle φ:
     *   endpoint = (pose_x + r·cos(φ+θ), pose_y + r·sin(φ+θ))
     * Bresenham ray-cast marks free cells; endpoint cell is marked occupied.
     *
     * @param ranges       Range readings [m]
     * @param angle_min    Start angle [rad]
     * @param angle_inc    Angular increment per reading [rad]
     * @param px, py, pth  Robot pose in global frame
     */
    void updateFromRanges(const std::vector<float>& ranges,
                          double angle_min, double angle_inc,
                          double px, double py, double pth);

    /// Build ROS OccupancyGrid message from current log-odds grid
    nav_msgs::msg::OccupancyGrid buildOccupancyGrid(rclcpp::Time stamp) const;

    void publishMap(rclcpp::Time stamp);

    // Public grid dimensions (used externally)
    int    width_{0}, height_{0};
    double resolution_{0.05};

private:
    double origin_x_{0.0}, origin_y_{0.0};
    std::vector<float> log_odds_;  ///< log-odds grid, size width×height

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;

    // Log-Odds update values (tunable)
    static constexpr float kLogOddsFree = -0.4f;
    static constexpr float kLogOddsOcc  =  0.85f;
    static constexpr float kLogOddsMax  =  5.0f;
    static constexpr float kThreshOcc   =  0.5f;
    static constexpr float kThreshFree  = -0.5f;

    void worldToGrid(double wx, double wy, int& gx, int& gy) const;

    bool inBounds(int gx, int gy) const;

    /// Bresenham ray-cast: mark cells from (x0,y0) to (x1,y1) with log_delta
    void bresenham(int x0, int y0, int x1, int y1, float log_delta);
};

}  // namespace slam

#endif  // SLAM_GRAPH_BASED_MAP_BUILDER_H
