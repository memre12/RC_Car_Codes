#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <limits>
#include <vector>

/**
 * Mission planner: decides between centerline and lattice mode.
 *
 * Checks a corridor of `path_width` around the centerline, up to
 * `lookahead_distance` ahead of the car, against the local occupancy
 * grid. If any cell in that corridor exceeds `obstacle_threshold` the
 * lattice planner is engaged; otherwise the car follows the centerline.
 *
 * The decision (true = centerline) is published at 10 Hz on the switch
 * topic so downstream nodes always know the current mode.
 */
class MissionPlanner : public rclcpp::Node
{
public:
    MissionPlanner() : Node("mission_planner")
    {
        const auto grid_topic = this->declare_parameter<std::string>("grid_topic", "/occupancy_grid");
        const auto path_topic = this->declare_parameter<std::string>("path_topic", "/path");
        const auto odom_topic = this->declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
        const auto switch_topic = this->declare_parameter<std::string>("switch_topic", "/switch_to_centerline");

        obstacle_threshold_ = this->declare_parameter<double>("obstacle_threshold", 50.0);
        lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 5.0);
        path_width_ = this->declare_parameter<double>("path_width", 1.5);

        grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            grid_topic, 10,
            std::bind(&MissionPlanner::gridCallback, this, std::placeholders::_1));
        // The centerline publisher latches its path (transient local): match
        // it so the path is received even if this node starts afterwards.
        rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
        latched_qos.transient_local().reliable();
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic, latched_qos,
            std::bind(&MissionPlanner::pathCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&MissionPlanner::odomCallback, this, std::placeholders::_1));

        switch_pub_ = this->create_publisher<std_msgs::msg::Bool>(switch_topic, 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MissionPlanner::timerCallback, this));

        RCLCPP_INFO(this->get_logger(),
                    "Mission planner started: corridor %.2f m wide, %.1f m ahead, threshold %.0f",
                    path_width_, lookahead_distance_, obstacle_threshold_);
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        centerline_path_ = msg;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = msg;
    }

    void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        occupancy_grid_ = msg;
        checkPathClearance();
    }

    /// Circular mask of (di, dj) cell offsets within path_width_/2, rebuilt
    /// only when the grid resolution changes.
    void updateCorridorMask(double resolution)
    {
        if (resolution == mask_resolution_)
            return;
        mask_resolution_ = resolution;
        corridor_mask_.clear();

        const double radius = path_width_ / 2.0;
        const int radius_cells = static_cast<int>(std::ceil(radius / resolution));
        const double radius_sq = radius * radius;
        for (int di = -radius_cells; di <= radius_cells; ++di)
        {
            for (int dj = -radius_cells; dj <= radius_cells; ++dj)
            {
                if ((di * di + dj * dj) * resolution * resolution <= radius_sq)
                    corridor_mask_.push_back({di, dj});
            }
        }
    }

    void checkPathClearance()
    {
        if (!occupancy_grid_ || !centerline_path_ || !current_odom_ ||
            centerline_path_->poses.empty())
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for inputs (grid=%d path=%d odom=%d)",
                                 occupancy_grid_ != nullptr, centerline_path_ != nullptr,
                                 current_odom_ != nullptr);
            return;
        }

        // Grid is in base_link frame, centered on the robot.
        const int width = occupancy_grid_->info.width;
        const int height = occupancy_grid_->info.height;
        const double resolution = occupancy_grid_->info.resolution;
        const double origin_x = occupancy_grid_->info.origin.position.x;
        const double origin_y = occupancy_grid_->info.origin.position.y;
        updateCorridorMask(resolution);

        const double robot_x = current_odom_->pose.pose.position.x;
        const double robot_y = current_odom_->pose.pose.position.y;
        const double robot_yaw = tf2::getYaw(current_odom_->pose.pose.orientation);
        const double cos_yaw = std::cos(robot_yaw);
        const double sin_yaw = std::sin(robot_yaw);

        // Nearest centerline point to the robot.
        size_t closest_idx = 0;
        double min_dist_sq = std::numeric_limits<double>::max();
        for (size_t i = 0; i < centerline_path_->poses.size(); ++i)
        {
            const double dx = centerline_path_->poses[i].pose.position.x - robot_x;
            const double dy = centerline_path_->poses[i].pose.position.y - robot_y;
            const double dist_sq = dx * dx + dy * dy;
            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                closest_idx = i;
            }
        }

        // Walk the corridor ahead of the car and look for lethal cells.
        const double lookahead_sq = lookahead_distance_ * lookahead_distance_;
        bool obstacle_detected = false;

        for (size_t i = closest_idx;
             i < centerline_path_->poses.size() && !obstacle_detected; ++i)
        {
            const double dx = centerline_path_->poses[i].pose.position.x - robot_x;
            const double dy = centerline_path_->poses[i].pose.position.y - robot_y;
            // Global (odom) -> robot (base_link) frame.
            const double local_x = dx * cos_yaw + dy * sin_yaw;
            const double local_y = -dx * sin_yaw + dy * cos_yaw;

            if (local_x < 0.0) // behind the car
                continue;
            if (local_x * local_x + local_y * local_y > lookahead_sq)
                break;

            const int center_i = static_cast<int>((local_x - origin_x) / resolution);
            const int center_j = static_cast<int>((local_y - origin_y) / resolution);

            for (const auto &offset : corridor_mask_)
            {
                const int ci = center_i + offset.di;
                const int cj = center_j + offset.dj;
                if (ci < 0 || ci >= width || cj < 0 || cj >= height)
                    continue;

                const size_t index = static_cast<size_t>(cj) * width + ci;
                if (index < occupancy_grid_->data.size() &&
                    occupancy_grid_->data[index] > obstacle_threshold_)
                {
                    obstacle_detected = true;
                    break;
                }
            }
        }

        const bool new_state = !obstacle_detected; // true = clear = centerline
        if (new_state != path_clear_)
        {
            path_clear_ = new_state;
            RCLCPP_INFO(this->get_logger(), "%s",
                        path_clear_ ? "Path is CLEAR - switching to CENTERLINE mode"
                                    : "OBSTACLE detected - switching to LATTICE mode");
        }
    }

    void timerCallback()
    {
        std_msgs::msg::Bool msg;
        msg.data = path_clear_; // true = centerline, false = lattice
        switch_pub_->publish(msg);
    }

    struct Offset
    {
        int di;
        int dj;
    };

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr switch_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;
    nav_msgs::msg::Path::SharedPtr centerline_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;

    double obstacle_threshold_;
    double lookahead_distance_;
    double path_width_;

    std::vector<Offset> corridor_mask_;
    double mask_resolution_ = -1.0;
    bool path_clear_ = true; // true = centerline mode, false = lattice mode
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionPlanner>());
    rclcpp::shutdown();
    return 0;
}
