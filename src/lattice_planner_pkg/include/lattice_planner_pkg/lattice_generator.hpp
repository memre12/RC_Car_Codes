#ifndef LATTICE_PLANNER_PKG__LATTICE_GENERATOR_HPP_
#define LATTICE_PLANNER_PKG__LATTICE_GENERATOR_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace Lattice
{

struct Point
{
    double x;
    double y;
};

struct Path
{
    std::vector<Point> points;
    double lateral_offset; // meters, signed, relative to the centerline
    double cost;
    bool blocked;
};

/**
 * Lattice trajectory generator.
 *
 * Works entirely in the robot frame (robot at the origin, x forward):
 * picks an anchor point on the local centerline at a speed-adaptive
 * lookahead distance, spawns candidate goal points laterally offset from
 * it, connects each with a cubic Hermite spline and collision-checks the
 * splines against a local occupancy grid.
 *
 * Selection policy (in priority order):
 *   1. If the centerline candidate (offset 0) is clear, take it.
 *   2. Otherwise stick with the previously chosen lateral offset if it
 *      is still clear (avoids oscillating between sides).
 *   3. Otherwise pick the cheapest clear candidate (cost = |offset| plus
 *      a penalty for jumping far from the current offset).
 *   4. If everything is blocked, fall back to the centerline.
 *
 * Not thread-safe; call setGridInfo / setCenterline / computeTrajectory
 * from a single thread (the default single-threaded executor is fine).
 */
class Generator
{
public:
    Generator() = default;

    /**
     * Point the generator at a local occupancy grid.
     * The grid is assumed centered on the robot. `grid_data` is borrowed,
     * not copied: the caller must keep the underlying message alive until
     * the next setGridInfo() call.
     */
    void setGridInfo(int width, int height, double resolution,
                     const std::vector<int8_t> &grid_data)
    {
        grid_width_ = width;
        grid_height_ = height;
        resolution_ = resolution;
        grid_data_ = &grid_data;
    }

    /// Centerline in the robot frame, ordered along the driving direction.
    void setCenterline(std::vector<Point> centerline)
    {
        centerline_ = std::move(centerline);
    }

    void setSpeedLimits(double min_speed, double max_speed)
    {
        min_speed_ = min_speed;
        max_speed_ = max_speed;
    }

    void setLookaheadDistances(double min_lookahead, double max_lookahead)
    {
        min_lookahead_ = min_lookahead;
        max_lookahead_ = max_lookahead;
    }

    void setLethalThreshold(int threshold) { lethal_threshold_ = threshold; }
    void setPathResolution(double resolution) { path_resolution_ = resolution; }
    void setCandidateOffsets(std::vector<double> offsets)
    {
        candidate_deltas_ = std::move(offsets);
    }

    /**
     * Generate candidates and return the selected trajectory (robot frame).
     * Returns an empty vector only if no centerline is available.
     */
    std::vector<Point> computeTrajectory(double current_speed)
    {
        last_candidates_.clear();
        if (centerline_.empty())
            return {};

        updateLookahead(current_speed);

        const size_t anchor_idx = findAnchorIndex();
        const double road_yaw = estimateRoadYaw(anchor_idx);
        const Point &anchor = centerline_[anchor_idx];

        // Unit normal of the road direction: lateral offsets are applied
        // along it.
        const double nx = -std::sin(road_yaw);
        const double ny = std::cos(road_yaw);

        last_candidates_.reserve(candidate_deltas_.size());
        for (const double delta : candidate_deltas_)
        {
            Path path;
            path.lateral_offset = delta;
            path.cost = std::abs(delta); // prefer staying near the centerline
            generateCubicSpline(anchor.x + nx * delta, anchor.y + ny * delta,
                                road_yaw, path.points);
            path.blocked = isPathBlocked(path.points);
            last_candidates_.push_back(std::move(path));
        }

        return selectBestCandidate();
    }

    /// Candidates from the last computeTrajectory() call, for visualization.
    const std::vector<Path> &getAllTrajectories() const { return last_candidates_; }

private:
    void updateLookahead(double current_speed)
    {
        const double clamped = std::clamp(current_speed, min_speed_, max_speed_);
        const double ratio = (clamped - min_speed_) / (max_speed_ - min_speed_);
        lookahead_distance_ = min_lookahead_ + ratio * (max_lookahead_ - min_lookahead_);
    }

    /// Centerline point (in front of the robot) whose distance from the
    /// robot is closest to the current lookahead distance.
    size_t findAnchorIndex() const
    {
        size_t target_idx = 0;
        double min_diff = std::numeric_limits<double>::max();
        for (size_t i = 0; i < centerline_.size(); ++i)
        {
            if (centerline_[i].x <= 0.0)
                continue;
            const double dist = std::hypot(centerline_[i].x, centerline_[i].y);
            const double diff = std::abs(dist - lookahead_distance_);
            if (diff < min_diff)
            {
                min_diff = diff;
                target_idx = i;
            }
        }
        return target_idx;
    }

    double estimateRoadYaw(size_t idx) const
    {
        if (idx + 1 < centerline_.size())
        {
            return std::atan2(centerline_[idx + 1].y - centerline_[idx].y,
                              centerline_[idx + 1].x - centerline_[idx].x);
        }
        if (idx > 0)
        {
            return std::atan2(centerline_[idx].y - centerline_[idx - 1].y,
                              centerline_[idx].x - centerline_[idx - 1].x);
        }
        return 0.0;
    }

    std::vector<Point> selectBestCandidate()
    {
        // 1. Strict priority: a clear centerline wins immediately.
        int centerline_idx = -1;
        for (size_t i = 0; i < last_candidates_.size(); ++i)
        {
            if (std::abs(last_candidates_[i].lateral_offset) < 1e-3)
            {
                centerline_idx = static_cast<int>(i);
                break;
            }
        }
        if (centerline_idx != -1 && !last_candidates_[centerline_idx].blocked)
        {
            locked_delta_ = 0.0;
            return last_candidates_[centerline_idx].points;
        }

        // 2. Centerline is blocked: keep the previously locked offset if it
        //    is still clear, for temporal consistency.
        for (const auto &candidate : last_candidates_)
        {
            if (!candidate.blocked &&
                std::abs(candidate.lateral_offset - locked_delta_) < 0.1)
            {
                return candidate.points;
            }
        }

        // 3. Cheapest clear candidate, biased towards the current side.
        int best_idx = -1;
        double best_cost = std::numeric_limits<double>::max();
        for (size_t i = 0; i < last_candidates_.size(); ++i)
        {
            const auto &candidate = last_candidates_[i];
            if (candidate.blocked)
                continue;

            double cost = candidate.cost;
            const bool same_side = (locked_delta_ * candidate.lateral_offset) > 0.0;
            if (same_side && std::abs(locked_delta_) > 0.01)
                cost *= 0.8;
            cost += std::abs(candidate.lateral_offset - locked_delta_);

            if (cost < best_cost)
            {
                best_cost = cost;
                best_idx = static_cast<int>(i);
            }
        }
        if (best_idx != -1)
        {
            locked_delta_ = last_candidates_[best_idx].lateral_offset;
            return last_candidates_[best_idx].points;
        }

        // 4. Everything blocked (possibly a false positive): fall back to
        //    the centerline rather than stopping dead.
        if (centerline_idx != -1)
            return last_candidates_[centerline_idx].points;

        return {};
    }

    /// Cubic Hermite spline from the robot pose (origin, heading +x) to the
    /// goal point with the road heading as the exit tangent.
    void generateCubicSpline(double gx, double gy, double gyaw,
                             std::vector<Point> &out_points) const
    {
        out_points.clear();
        const double dist = std::hypot(gx, gy);
        const double scale = dist * 1.2; // tangent magnitude

        const double mx0 = scale;
        const double my0 = 0.0;
        const double mx1 = scale * std::cos(gyaw);
        const double my1 = scale * std::sin(gyaw);

        const int steps = std::max(5, static_cast<int>(dist / path_resolution_));
        out_points.reserve(steps + 1);

        for (int i = 0; i <= steps; ++i)
        {
            const double t = static_cast<double>(i) / steps;
            const double t2 = t * t;
            const double t3 = t2 * t;

            const double h10 = t3 - 2.0 * t2 + t;
            const double h01 = -2.0 * t3 + 3.0 * t2;
            const double h11 = t3 - t2;

            out_points.push_back({h10 * mx0 + h01 * gx + h11 * mx1,
                                  h10 * my0 + h01 * gy + h11 * my1});
        }
    }

    /// Collision check against the local grid (robot at the grid center).
    bool isPathBlocked(const std::vector<Point> &points) const
    {
        if (!grid_data_)
            return false; // no grid yet: assume clear

        const int center_x = grid_width_ / 2;
        const int center_y = grid_height_ / 2;
        const double inv_res = 1.0 / resolution_;

        for (const auto &pt : points)
        {
            const int idx_x = center_x + static_cast<int>(pt.x * inv_res);
            const int idx_y = center_y + static_cast<int>(pt.y * inv_res);

            if (idx_x < 0 || idx_x >= grid_width_ || idx_y < 0 || idx_y >= grid_height_)
                continue;

            const size_t index = static_cast<size_t>(idx_y) * grid_width_ + idx_x;
            if (index < grid_data_->size() && (*grid_data_)[index] > lethal_threshold_)
                return true;
        }
        return false;
    }

    // -- Grid (borrowed) ------------------------------------------------------
    int grid_width_ = 0;
    int grid_height_ = 0;
    double resolution_ = 0.05;
    const std::vector<int8_t> *grid_data_ = nullptr;

    // -- Inputs / outputs -------------------------------------------------------
    std::vector<Point> centerline_;
    std::vector<Path> last_candidates_;

    // -- Tuning ------------------------------------------------------------------
    double min_lookahead_ = 1.0;
    double max_lookahead_ = 4.0;
    double min_speed_ = 0.5;
    double max_speed_ = 5.0;
    double lookahead_distance_ = 1.0;
    double path_resolution_ = 0.05; // meters between spline samples
    std::vector<double> candidate_deltas_ =
        {-1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
    int lethal_threshold_ = 50;

    // -- Selection state -----------------------------------------------------------
    double locked_delta_ = 0.0; // lateral offset currently committed to
};

} // namespace Lattice

#endif // LATTICE_PLANNER_PKG__LATTICE_GENERATOR_HPP_
