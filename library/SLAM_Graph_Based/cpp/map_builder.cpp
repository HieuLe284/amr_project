#include "map_builder.h"

slam::MapBuilder::MapBuilder(double resolution, int width, int height,
                       double origin_x, double origin_y)
    : resolution_(resolution), width_(width), height_(height),
      origin_x_(origin_x), origin_y_(origin_y),
      log_odds_(width * height, 0.0f)
{}

void slam::MapBuilder::setPublisher(
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub)
{
    pub_ = pub;
}

void slam::MapBuilder::clearMap() {
    std::fill(log_odds_.begin(), log_odds_.end(), 0.0f);
}

void slam::MapBuilder::updateFromRanges(const std::vector<float>& ranges,
                                  double angle_min, double angle_inc,
                                  double px, double py, double pth)
{
    for (size_t i = 0; i < ranges.size(); ++i) {
        float r = ranges[i];
        if (!std::isfinite(r) || r < 0.1f || r > 30.0f) continue;

        double angle = angle_min + i * angle_inc + pth;
        double ex = px + r * std::cos(angle);
        double ey = py + r * std::sin(angle);

        // Source cell
        int sx, sy;
        worldToGrid(px, py, sx, sy);

        // Endpoint cell
        int ex_cell, ey_cell;
        worldToGrid(ex, ey, ex_cell, ey_cell);

        // Ray-cast: free cells along beam
        bresenham(sx, sy, ex_cell, ey_cell, kLogOddsFree);

        // Mark endpoint occupied
        if (inBounds(ex_cell, ey_cell))
            log_odds_[ex_cell + ey_cell * width_] =
                std::min(log_odds_[ex_cell + ey_cell * width_] + kLogOddsOcc,
                         kLogOddsMax);
    }
}

nav_msgs::msg::OccupancyGrid slam::MapBuilder::buildOccupancyGrid(rclcpp::Time stamp) const {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = "map";
    msg.info.resolution = static_cast<float>(resolution_);
    msg.info.width      = width_;
    msg.info.height     = height_;
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(width_ * height_);
    for (int i = 0; i < width_ * height_; ++i) {
        float l = log_odds_[i];
        if (l >  kThreshOcc)  msg.data[i] = 100;
        else if (l < kThreshFree) msg.data[i] = 0;
        else                  msg.data[i] = -1;
    }
    return msg;
}

void slam::MapBuilder::publishMap(rclcpp::Time stamp) {
    if (pub_) pub_->publish(buildOccupancyGrid(stamp));
}

void slam::MapBuilder::worldToGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>((wx - origin_x_) / resolution_);
    gy = static_cast<int>((wy - origin_y_) / resolution_);
}

bool slam::MapBuilder::inBounds(int gx, int gy) const {
    return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

void slam::MapBuilder::bresenham(int x0, int y0, int x1, int y1, float log_delta) {
    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (true) {
        if (x0 == x1 && y0 == y1) break;   // stop before endpoint
        if (inBounds(x0, y0)) {
            float& l = log_odds_[x0 + y0 * width_];
            l = std::max(-kLogOddsMax, l + log_delta);
        }
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}