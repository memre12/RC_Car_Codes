#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>

class Controller : public rclcpp::Node
{
public:
    Controller()
        : Node("controller_cpp"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // Declare parameters (speeds in m/s)
        this->declare_parameter<double>("max_speed", 0.9);
        this->declare_parameter<double>("min_speed", 0.6);
        this->declare_parameter<double>("brake_distance", 1.5);
        this->declare_parameter<double>("Kp", 0.5);
        this->declare_parameter<double>("Kd", 5.0);
        this->declare_parameter<double>("Ts", 0.05);
        this->declare_parameter<bool>("speed_from_path", true);
        this->declare_parameter<double>("pd_max", 0.05);
        this->declare_parameter<std::string>("trajectory_topic", "/path");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<double>("Ld", 5.0);
        this->declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
        this->declare_parameter<std::string>("switch_topic", "/switch_to_centerline");

        // Get parameters;
        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        brake_distance_ = this->get_parameter("brake_distance").as_double();
        Kp_ = this->get_parameter("Kp").as_double();
        Kd_ = this->get_parameter("Kd").as_double();
        Ts_ = this->get_parameter("Ts").as_double();
        speed_from_path = this->get_parameter("speed_from_path").as_bool();
        pd_max = this->get_parameter("pd_max").as_double();
        trajectory_topic_ = this->get_parameter("trajectory_topic").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        Ld = this->get_parameter("Ld").as_double();
        std::string odom_topic_ = this->get_parameter("odom_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "---- desired_speed_pub params ----");
        RCLCPP_INFO(this->get_logger(), "trajectory_topic : %s", trajectory_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "base_frame       : %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "max_speed        : %.3f", max_speed_);
        RCLCPP_INFO(this->get_logger(), "min_speed        : %.3f", min_speed_);
        RCLCPP_INFO(this->get_logger(), "brake_distance   : %.3f", brake_distance_);
        RCLCPP_INFO(this->get_logger(), "Kp               : %.3f", Kp_);
        RCLCPP_INFO(this->get_logger(), "Kd               : %.3f", Kd_);
        RCLCPP_INFO(this->get_logger(), "Ts               : %.3f", Ts_);
        RCLCPP_INFO(this->get_logger(), "speed_from_path               : %d", speed_from_path);
        RCLCPP_INFO(this->get_logger(), "pd_max               : %.3f", pd_max);
        RCLCPP_INFO(this->get_logger(), "Ld               : %.3f", Ld);

        // Initialize state
        Ukd_prev_ = 0.0;
        error_prev_ = 0.0;
        reference_speed_ = 0.0;
        last_ukd = 0.0;
        last_vref = 0.0;
        // Publishers and Subscribers
        vref_pub_ = this->create_publisher<std_msgs::msg::Float64>("/target_speed", 10);

        transformed_path_pub = this->create_publisher<nav_msgs::msg::Path>("/transformed_path", 10);

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            trajectory_topic_,
            rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&Controller::vrefGenerator, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&Controller::odomCallback, this, std::placeholders::_1));

        // Mission planner mode: while the lattice planner is avoiding an
        // obstacle, force the speed reference down to min_speed.
        switch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            this->get_parameter("switch_topic").as_string(), 10,
            std::bind(&Controller::switchCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(Ts_),
            std::bind(&Controller::timerCallback, this));
    }

private:
    double computeDynamicDerivative(const nav_msgs::msg::Path &path_transformed)
    {
        if (path_transformed.poses.size() < 2)
        {
            return this->last_ukd;
        }

        double sum_dy = 0.0;
        int count = 0;

        for (size_t i = 1; i < path_transformed.poses.size(); ++i)
        {
            double dy = std::abs(path_transformed.poses[i].pose.position.y -
                                 path_transformed.poses[i - 1].pose.position.y);
            double dx = std::abs(path_transformed.poses[i].pose.position.x -
                                 path_transformed.poses[i - 1].pose.position.x);
            double local_slope = (dx > 0.001) ? dy / dx : 0.0;
            sum_dy += local_slope * local_slope;
            count++;
        }

        double mean_dy = (count > 0) ? std::sqrt(sum_dy / count) : 0.0;

        double Ukd = Kd_ * mean_dy;
        this->last_ukd = Ukd;
        return Ukd;
    }

    nav_msgs::msg::Path transformPath(const nav_msgs::msg::Path &path_msg)
    {
        nav_msgs::msg::Path out;
        out.header.frame_id = "base_link";
        out.header.stamp = this->get_clock()->now();

        for (const auto &pose_stamped : path_msg.poses)
        {
            geometry_msgs::msg::PointStamped pin, pout;
            pin.header = pose_stamped.header;
            pin.point = pose_stamped.pose.position;

            try
            {
                geometry_msgs::msg::TransformStamped tf =
                    tf_buffer_.lookupTransform(
                        "base_link",
                        pin.header.frame_id,
                        tf2::TimePointZero,
                        tf2::durationFromSec(0.05));

                tf2::doTransform(pin, pout, tf);

                geometry_msgs::msg::PoseStamped p2;
                p2.header = pout.header;
                p2.pose.position = pout.point;
                p2.pose.orientation = pose_stamped.pose.orientation;

                out.poses.push_back(p2);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            }
        }

        return out;
    }

    void switchCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // true = centerline mode (clear), false = lattice mode (obstacle)
        const bool use_lattice = !msg->data;
        if (use_lattice != use_lattice_)
        {
            use_lattice_ = use_lattice;
            RCLCPP_INFO(this->get_logger(), "%s",
                        use_lattice_
                            ? "LATTICE mode: speed reference forced to min_speed"
                            : "CENTERLINE mode: normal speed reference");
        }
    }

    void vrefGenerator(const nav_msgs::msg::Path::SharedPtr msg)
    {
        static bool do_not_take_again = false;
        if (!do_not_take_again)
        {

            latest_path_ = *msg;
            do_not_take_again = true;
            RCLCPP_INFO(this->get_logger(),
                        "path msg is subscribed");
        }
    }

    int findClosestPointIndex(const nav_msgs::msg::Path &path)
    {
        if (path.poses.empty())
            return -1;

        double cx = current_pose_.position.x;
        double cy = current_pose_.position.y;

        double min_dist = 999999.0;
        int min_index = -1;

        for (int i = 0; i < (int)path.poses.size(); i++)
        {
            double px = path.poses[i].pose.position.x;
            double py = path.poses[i].pose.position.y;

            double dx = px - cx;
            double dy = py - cy;

            double dist = dx * dx + dy * dy;

            if (dist < min_dist)
            {
                min_dist = dist;
                min_index = i;
            }
        }
        return min_index;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        is_odom_received = true;

        if (latest_path_.poses.empty())
            return;

        if (speed_from_path)
        {
            int idx = findClosestPointIndex(latest_path_);

            if (idx < 0)
                return;

            double speed_cmd = latest_path_.poses[idx].pose.orientation.w;
            if (speed_cmd > max_speed_)
            {
                speed_cmd = max_speed_;
            }
            if (use_lattice_)
            {
                speed_cmd = min_speed_; // obstacle avoidance: slow down
            }

            std_msgs::msg::Float64 msg_out;
            msg_out.data = speed_cmd;
            vref_pub_->publish(msg_out);

            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "Closest idx=%d | data_speed=%.2f",
                idx, speed_cmd);
        }
        else
        {
            // Speed is calculating dynamically.
        }
    }
    void timerCallback()
    {
        if (latest_path_.poses.empty())
        {
            return;
        }

        if (!is_odom_received)
        {
            return;
        }

        auto path_transformed = transformPath(latest_path_);
        if (path_transformed.poses.empty())
        {
            return;
        }

        if (!speed_from_path)
        {
            std::vector<geometry_msgs::msg::PoseStamped> filtered_poses;
            filtered_poses.reserve(path_transformed.poses.size());

            double best_dist = 1e9;
            size_t closest_idx = 0;
            bool found_front_point = false;

            for (size_t i = 0; i < path_transformed.poses.size(); i++)
            {
                double dx = path_transformed.poses[i].pose.position.x;
                double dy = path_transformed.poses[i].pose.position.y;

                if (dx <= 0.0)
                    continue;

                double d = std::sqrt(dx * dx + dy * dy);

                if (d < best_dist)
                {
                    best_dist = d;
                    closest_idx = i;
                    found_front_point = true;
                }
            }

            if (!found_front_point)
            {
                std_msgs::msg::Float64 msg;
                msg.data = this->min_speed_;
                vref_pub_->publish(msg);
            }
            else
            {

                size_t N = path_transformed.poses.size();

                double last_dx = 0.0;
                double last_dy = 0.0;
                double last_yaw = 0.0;
                bool first = true;

                for (size_t step = 0; step < N; step++)
                {
                    size_t i = (closest_idx + step) % N;

                    const auto &p = path_transformed.poses[i];
                    double dx = p.pose.position.x;
                    double dy = p.pose.position.y;

                    double dist = std::sqrt(dx * dx + dy * dy);

                    // todo: use encoder speed
                    if (dist > (brake_distance_ * ((last_vref / max_speed_) * (last_vref / max_speed_) + 1)))
                        break;

                    double ddx = dx - last_dx;
                    double ddy = dy - last_dy;

                    double yaw = std::atan2(ddy, ddx);
                    double dyaw = std::fabs(yaw - last_yaw);
                    if (!first)
                    {
                        if (std::fabs(ddx) > 1.0)
                            break;
                        if (std::fabs(ddy) > 1.0)
                            break;
                        if (dyaw > M_PI)
                            break;
                    }
                    filtered_poses.push_back(p);

                    double ddy2 = dy - last_dy;
                    double ddx2 = dx - last_dx;
                    last_yaw = std::atan2(ddy2, ddx2);
                    last_dx = dx;
                    last_dy = dy;
                    first = false;
                }
            }

            double error = 0.0;

            if (filtered_poses.size() < 3)
            {
                reference_speed_ = min_speed_;
                std_msgs::msg::Float64 msg;
                msg.data = reference_speed_;
                vref_pub_->publish(msg);
                return;
            }

            for (const auto &p : filtered_poses)
            {
                double y = p.pose.position.y;
                error += y * y;
            }

            error = std::sqrt(error);

            nav_msgs::msg::Path filtered_path;
            filtered_path.header = path_transformed.header;
            filtered_path.poses = std::move(filtered_poses);
            transformed_path_pub->publish(filtered_path);

            double Ukp = Kp_ * error;
            double Ukd = computeDynamicDerivative(filtered_path);
            double pd_output = Ukp + Ukd;

            // EMA try it not sure
            if (pd_output > pd_max)
                pd_max = 0.95 * pd_max + 0.05 * std::abs(pd_output);
            else
                pd_max *= 0.999;
            double pd_norm = pd_output / pd_max;
            pd_norm = std::clamp(pd_norm, -1.0, 1.0);

            double t = (pd_norm + 1.0) * 0.5; // [-1,1] → [0,1]
            reference_speed_ = max_speed_ - t * (max_speed_ - min_speed_);
            if (use_lattice_)
            {
                reference_speed_ = min_speed_; // obstacle avoidance: slow down
            }
            last_vref = reference_speed_;
            std_msgs::msg::Float64 msg;
            msg.data = reference_speed_;
            vref_pub_->publish(msg);
        }
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vref_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr switch_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr transformed_path_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path latest_path_;
    geometry_msgs::msg::Pose current_pose_;
    bool speed_from_path;
    bool is_odom_received = false;
    bool use_lattice_ = false; // true while the lattice planner is active
    double pd_max;
    double Ld;

    // Params
    std::string trajectory_topic_;
    std::string base_frame_;
    double max_speed_, min_speed_, brake_distance_, Kp_, Kd_, Ts_;

    // State vars
    double Ukd_prev_, error_prev_, reference_speed_, last_ukd, last_vref;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
