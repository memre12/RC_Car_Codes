#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

/**
 * Converts laser scans into a local occupancy grid centered on the robot
 * (base_link). Only obstacle endpoints are marked (no ray tracing): the
 * lattice planner just needs to know which cells are lethal.
 *
 * Optimized for the Jetson:
 *   - sin/cos of the beam angles are computed once and cached (they only
 *     change if the scan geometry changes),
 *   - the circular inflation mask is precomputed as an offset list,
 *   - the grid buffer is reused between scans instead of reallocated.
 */
class ScanToGrid : public rclcpp::Node
{
public:
    ScanToGrid() : Node("scan_to_grid")
    {
        grid_res_ = this->declare_parameter<double>("grid_resolution", 0.05);
        const double size_x = this->declare_parameter<double>("grid_size_x", 6.0); // meters
        const double size_y = this->declare_parameter<double>("grid_size_y", 6.0); // meters
        // Should be at least half the vehicle width (0.158 m for this car).
        inflation_radius_ = this->declare_parameter<double>("inflation_radius", 0.15);
        inflation_cost_ = static_cast<int8_t>(this->declare_parameter<int>("inflation_cost", 80));
        const auto scan_topic = this->declare_parameter<std::string>("scan_topic", "/scan");
        const auto grid_topic = this->declare_parameter<std::string>("grid_topic", "/occupancy_grid");

        grid_width_ = static_cast<int>(size_x / grid_res_);
        grid_height_ = static_cast<int>(size_y / grid_res_);
        origin_x_ = -(grid_width_ * grid_res_) / 2.0;
        origin_y_ = -(grid_height_ * grid_res_) / 2.0;
        grid_data_.resize(static_cast<size_t>(grid_width_) * grid_height_, 0);

        buildInflationMask();

        // Best-effort QoS for high-rate laser scans.
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, rclcpp::SensorDataQoS(),
            std::bind(&ScanToGrid::scanCallback, this, std::placeholders::_1));
        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic, 10);

        RCLCPP_INFO(this->get_logger(),
                    "ScanToGrid started: grid %dx%d @ %.2f m, inflation %.2f m (%zu cells)",
                    grid_width_, grid_height_, grid_res_, inflation_radius_,
                    inflation_offsets_.size());
    }

private:
    /// Circular mask of (dx, dy) cell offsets within inflation_radius_.
    void buildInflationMask()
    {
        const int radius_cells = static_cast<int>(std::ceil(inflation_radius_ / grid_res_));
        const double radius_sq = inflation_radius_ * inflation_radius_;
        for (int dx = -radius_cells; dx <= radius_cells; ++dx)
        {
            for (int dy = -radius_cells; dy <= radius_cells; ++dy)
            {
                const double dist_sq = (dx * dx + dy * dy) * grid_res_ * grid_res_;
                if (dist_sq <= radius_sq)
                    inflation_offsets_.push_back({dx, dy});
            }
        }
    }

    /// Recompute the per-beam sin/cos table when the scan geometry changes.
    void updateTrigCache(const sensor_msgs::msg::LaserScan &msg)
    {
        if (msg.angle_min == cached_angle_min_ &&
            msg.angle_increment == cached_angle_increment_ &&
            msg.ranges.size() == cos_cache_.size())
        {
            return;
        }

        cached_angle_min_ = msg.angle_min;
        cached_angle_increment_ = msg.angle_increment;
        cos_cache_.resize(msg.ranges.size());
        sin_cache_.resize(msg.ranges.size());
        double angle = msg.angle_min;
        for (size_t i = 0; i < msg.ranges.size(); ++i, angle += msg.angle_increment)
        {
            cos_cache_[i] = std::cos(angle);
            sin_cache_[i] = std::sin(angle);
        }
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        updateTrigCache(*msg);

        // Reuse the buffer: clear instead of reallocate.
        std::fill(grid_data_.begin(), grid_data_.end(), 0);

        const double inv_res = 1.0 / grid_res_;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            const float range = msg->ranges[i];
            if (std::isinf(range) || std::isnan(range) ||
                range < msg->range_min || range > msg->range_max)
            {
                continue;
            }

            // Beam endpoint in the sensor frame -> grid indices.
            const int idx_x = static_cast<int>((range * cos_cache_[i] - origin_x_) * inv_res);
            const int idx_y = static_cast<int>((range * sin_cache_[i] - origin_y_) * inv_res);
            if (idx_x < 0 || idx_x >= grid_width_ || idx_y < 0 || idx_y >= grid_height_)
                continue;

            grid_data_[static_cast<size_t>(idx_y) * grid_width_ + idx_x] = 100;

            for (const auto &offset : inflation_offsets_)
            {
                const int nx = idx_x + offset.dx;
                const int ny = idx_y + offset.dy;
                if (nx < 0 || nx >= grid_width_ || ny < 0 || ny >= grid_height_)
                    continue;
                int8_t &cell = grid_data_[static_cast<size_t>(ny) * grid_width_ + nx];
                if (cell < inflation_cost_)
                    cell = inflation_cost_;
            }
        }

        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header = msg->header;
        grid_msg.header.frame_id = "base_link"; // local grid centered on the robot
        grid_msg.info.resolution = grid_res_;
        grid_msg.info.width = grid_width_;
        grid_msg.info.height = grid_height_;
        grid_msg.info.origin.position.x = origin_x_;
        grid_msg.info.origin.position.y = origin_y_;
        grid_msg.info.origin.orientation.w = 1.0;
        grid_msg.data = grid_data_;
        grid_pub_->publish(grid_msg);
    }

    struct Offset
    {
        int dx;
        int dy;
    };

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    double grid_res_;
    int grid_width_, grid_height_;
    double origin_x_, origin_y_;
    double inflation_radius_;
    int8_t inflation_cost_;

    std::vector<int8_t> grid_data_;
    std::vector<Offset> inflation_offsets_;
    std::vector<double> cos_cache_, sin_cache_;
    float cached_angle_min_ = std::numeric_limits<float>::quiet_NaN();
    float cached_angle_increment_ = std::numeric_limits<float>::quiet_NaN();
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToGrid>());
    rclcpp::shutdown();
    return 0;
}
