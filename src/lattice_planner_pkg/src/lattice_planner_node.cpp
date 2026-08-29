#include "lattice_planner_pkg/lattice_generator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // foxy/humble compatibility

#include <cmath>
#include <limits>
#include <vector>

/**
 * Lattice planner node.
 *
 * Inputs:  global centerline path, localized odometry, local occupancy grid
 *          and the centerline/lattice switch from the mission planner.
 * Output:  the path the pure pursuit controller should follow.
 *
 * In centerline mode the global path is forwarded untouched. In lattice
 * mode a window of the centerline around the car is transformed into the
 * robot frame, handed to Lattice::Generator, and the selected avoidance
 * trajectory is transformed back into the global frame.
 *
 * Planning is rate-limited (planner_frequency) so a high-rate odometry
 * source does not make the node replan hundreds of times per second on
 * the Jetson.
 */
class LatticePlannerNode : public rclcpp::Node
{
public:
    LatticePlannerNode() : Node("lattice_planner")
    {
        // -- Topics -----------------------------------------------------------
        const auto path_topic = this->declare_parameter<std::string>("path_topic", "/path");
        const auto odom_topic = this->declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
        const auto grid_topic = this->declare_parameter<std::string>("grid_topic", "/occupancy_grid");
        const auto output_topic = this->declare_parameter<std::string>("output_path_topic", "/selected_path");
        const auto switch_topic = this->declare_parameter<std::string>("switch_topic", "/switch_to_centerline");

        // -- Planner tuning -----------------------------------------------------
        planner_frequency_ = this->declare_parameter<double>("planner_frequency", 20.0);
        planner_horizon_ = this->declare_parameter<double>("planner_horizon", 10.0);
        centerline_window_behind_ = this->declare_parameter<int>("centerline_window_behind", 10);
        centerline_window_ahead_ = this->declare_parameter<int>("centerline_window_ahead", 100);

        planner_.setSpeedLimits(
            this->declare_parameter<double>("min_speed", 0.5),
            this->declare_parameter<double>("max_speed", 10.0));
        planner_.setLookaheadDistances(
            this->declare_parameter<double>("min_lookahead", 3.0),
            this->declare_parameter<double>("max_lookahead", 15.0));
        planner_.setLethalThreshold(
            this->declare_parameter<int>("lethal_threshold", 50));
        planner_.setPathResolution(
            this->declare_parameter<double>("path_resolution", 0.05));
        planner_.setCandidateOffsets(
            this->declare_parameter<std::vector<double>>(
                "candidate_offsets",
                {-1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0}));

        // The centerline publisher latches its path (transient local).
        rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
        latched_qos.transient_local().reliable();

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic, latched_qos,
            std::bind(&LatticePlannerNode::pathCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&LatticePlannerNode::odomCallback, this, std::placeholders::_1));
        grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            grid_topic, 10,
            std::bind(&LatticePlannerNode::gridCallback, this, std::placeholders::_1));
        switch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            switch_topic, 10,
            std::bind(&LatticePlannerNode::switchCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(output_topic, 10);
        debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lattice_debug", 10);
        debug_closest_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/lattice_debug/closest_point", 10);

        last_plan_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
                    "Lattice planner started: %.0f Hz, horizon %.1f m, window [-%d, +%d] pts",
                    planner_frequency_, planner_horizon_,
                    centerline_window_behind_, centerline_window_ahead_);
    }

private:
    void switchCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        const bool use_centerline = msg->data;
        const bool was_lattice = use_lattice_;
        use_lattice_ = !use_centerline;

        // On the lattice -> centerline transition, immediately hand the
        // centerline back to the controller (the latched input path will
        // not be re-delivered on its own).
        if (was_lattice && use_centerline && global_path_)
        {
            path_pub_->publish(*global_path_);
            RCLCPP_INFO(this->get_logger(), "Switched to CENTERLINE mode");
        }
        else if (!was_lattice && use_lattice_)
        {
            RCLCPP_INFO(this->get_logger(), "Switched to LATTICE mode");
        }
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        global_path_ = msg;
        if (!use_lattice_)
            path_pub_->publish(*global_path_);
    }

    void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // Keep the message alive: the generator borrows msg->data.
        occupancy_grid_ = msg;
        planner_.setGridInfo(msg->info.width, msg->info.height,
                             msg->info.resolution, msg->data);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!use_lattice_)
            return;

        // Rate limit: odometry can arrive much faster than we want to plan.
        const auto now = this->now();
        if (planner_frequency_ > 0.0 &&
            (now - last_plan_time_).seconds() < 1.0 / planner_frequency_)
        {
            return;
        }

        if (!global_path_ || global_path_->poses.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for global path...");
            return;
        }
        if (!occupancy_grid_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for occupancy grid...");
            return;
        }
        last_plan_time_ = now;

        const double current_x = msg->pose.pose.position.x;
        const double current_y = msg->pose.pose.position.y;
        const double current_yaw = tf2::getYaw(msg->pose.pose.orientation);
        const double current_speed = msg->twist.twist.linear.x;
        const double cos_yaw = std::cos(current_yaw);
        const double sin_yaw = std::sin(current_yaw);

        // 1. Closest centerline point, ignoring points whose heading differs
        //    from the car's by more than 45 deg (rules out the opposite lane
        //    of the track on tight loops).
        const size_t closest_idx = findClosestPathIndex(current_x, current_y, current_yaw);
        publishClosestPointDebug(closest_idx);

        // 2. Window of the centerline around the car, in the robot frame.
        std::vector<Lattice::Point> local_centerline;
        const size_t n = global_path_->poses.size();
        const size_t behind = static_cast<size_t>(centerline_window_behind_);
        const size_t start_idx = (closest_idx > behind) ? closest_idx - behind : 0;
        const size_t end_idx = std::min(closest_idx + centerline_window_ahead_, n);
        local_centerline.reserve(end_idx - start_idx);

        for (size_t i = start_idx; i < end_idx; ++i)
        {
            const double dx = global_path_->poses[i].pose.position.x - current_x;
            const double dy = global_path_->poses[i].pose.position.y - current_y;
            // Global -> robot frame (rotate by -yaw).
            const double lx = dx * cos_yaw + dy * sin_yaw;
            const double ly = -dx * sin_yaw + dy * cos_yaw;
            if (lx > -2.0 && lx < planner_horizon_)
                local_centerline.push_back({lx, ly});
        }

        planner_.setCenterline(std::move(local_centerline));

        // 3. Plan in the robot frame.
        const std::vector<Lattice::Point> best_local_path = planner_.computeTrajectory(current_speed);

        publishCandidateDebug(current_x, current_y, cos_yaw, sin_yaw);

        if (best_local_path.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Lattice planner failed to find a path!");
            return;
        }

        // 4. Robot frame -> global frame, with orientations from the path
        //    direction, and hand it to the controller.
        publishPath(best_local_path, msg->header.frame_id,
                    current_x, current_y, cos_yaw, sin_yaw);
    }

    size_t findClosestPathIndex(double current_x, double current_y, double current_yaw) const
    {
        size_t closest_idx = 0;
        double min_dist_sq = std::numeric_limits<double>::max();

        for (size_t i = 0; i < global_path_->poses.size(); ++i)
        {
            const auto &pose = global_path_->poses[i].pose;

            double heading_diff = current_yaw - tf2::getYaw(pose.orientation);
            while (heading_diff > M_PI)
                heading_diff -= 2.0 * M_PI;
            while (heading_diff < -M_PI)
                heading_diff += 2.0 * M_PI;
            if (std::abs(heading_diff) > M_PI / 4.0)
                continue;

            const double dx = pose.position.x - current_x;
            const double dy = pose.position.y - current_y;
            const double dist_sq = dx * dx + dy * dy;
            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                closest_idx = i;
            }
        }
        return closest_idx;
    }

    void publishPath(const std::vector<Lattice::Point> &local_path,
                     const std::string &frame_id,
                     double current_x, double current_y,
                     double cos_yaw, double sin_yaw)
    {
        nav_msgs::msg::Path output_path;
        output_path.header.stamp = this->now();
        output_path.header.frame_id = frame_id;
        output_path.poses.reserve(local_path.size());

        for (const auto &pt : local_path)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = output_path.header;
            pose.pose.position.x = pt.x * cos_yaw - pt.y * sin_yaw + current_x;
            pose.pose.position.y = pt.x * sin_yaw + pt.y * cos_yaw + current_y;
            output_path.poses.push_back(pose);
        }

        for (size_t i = 0; output_path.poses.size() >= 2 && i < output_path.poses.size(); ++i)
        {
            const size_t from = (i + 1 < output_path.poses.size()) ? i : i - 1;
            const size_t to = (i + 1 < output_path.poses.size()) ? i + 1 : i;
            const double dx = output_path.poses[to].pose.position.x -
                              output_path.poses[from].pose.position.x;
            const double dy = output_path.poses[to].pose.position.y -
                              output_path.poses[from].pose.position.y;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, std::atan2(dy, dx));
            output_path.poses[i].pose.orientation = tf2::toMsg(q);
        }

        path_pub_->publish(output_path);
    }

    void publishClosestPointDebug(size_t closest_idx)
    {
        if (debug_closest_point_pub_->get_subscription_count() == 0)
            return;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.ns = "closest_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = global_path_->poses[closest_idx].pose;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.g = 1.0;
        marker.color.a = 1.0;
        debug_closest_point_pub_->publish(marker);
    }

    void publishCandidateDebug(double cx, double cy, double cos_yaw, double sin_yaw)
    {
        if (debug_pub_->get_subscription_count() == 0)
            return;

        visualization_msgs::msg::MarkerArray markers;
        int id = 0;

        for (const auto &path : planner_.getAllTrajectories())
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "odom";
            m.header.stamp = this->now();
            m.ns = "lattice_candidates";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.05;

            if (path.blocked)
            {
                m.color.r = 1.0;
                m.color.a = 0.3;
            }
            else
            {
                m.color.g = 1.0;
                m.color.b = 0.5;
                m.color.a = 0.5;
            }

            m.points.reserve(path.points.size());
            for (const auto &pt : path.points)
            {
                geometry_msgs::msg::Point p;
                p.x = pt.x * cos_yaw - pt.y * sin_yaw + cx;
                p.y = pt.x * sin_yaw + pt.y * cos_yaw + cy;
                m.points.push_back(p);
            }
            markers.markers.push_back(std::move(m));
        }
        debug_pub_->publish(markers);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr switch_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_closest_point_pub_;

    nav_msgs::msg::Path::SharedPtr global_path_;
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;

    Lattice::Generator planner_;
    rclcpp::Time last_plan_time_;

    double planner_frequency_;
    double planner_horizon_;
    int centerline_window_behind_;
    int centerline_window_ahead_;

    bool use_lattice_ = false; // false = centerline mode, true = lattice mode
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LatticePlannerNode>());
    rclcpp::shutdown();
    return 0;
}
