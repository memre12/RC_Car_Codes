#include "race_car_controller/race_car_controller.hpp"

#include <algorithm>
#include <limits>

namespace racecar
{

namespace
{
constexpr double kStopSpeedErpm = 0.0;
constexpr double kRadToDeg = 180.0 / M_PI;
} // namespace

RaceCarController::RaceCarController()
    : Node("racecar_controller")
{
    // -- Vehicle geometry (see race_car_parameters.txt) ----------------------
    wheelbase_ = this->declare_parameter<double>("wheelbase", 0.28);
    max_steer_angle_deg_ = this->declare_parameter<double>("max_steer_angle_deg", 29.85);

    // steering_gain = 1.0 is the pure geometric pure pursuit mapping.
    // The default 2.35 reproduces the response tuned on track with the old
    // implementation (gain 2.2 on a 45 deg scale with a 0.45 m wheelbase):
    // same servo slope per degree of heading error and the same saturation
    // point (full lock at delta = 12.7 deg).
    steering_gain_ = this->declare_parameter<double>("steering_gain", 2.35);

    servo_min_ = this->declare_parameter<double>("servo_min", 0.2);
    servo_max_ = this->declare_parameter<double>("servo_max", 0.8);
    servo_center_ = this->declare_parameter<double>("servo_center", 0.5);

    // -- Lookahead / speed (all speeds in m/s) ---------------------------------
    base_lookahead_ = this->declare_parameter<double>("lookahead_distance", 1.3);
    min_lookahead_ = this->declare_parameter<double>("min_look_ahead_distance", 0.6);
    max_lookahead_ = this->declare_parameter<double>("max_look_ahead_distance", 2.4);
    min_speed_ = this->declare_parameter<double>("min_speed", 0.6);
    max_speed_ = this->declare_parameter<double>("max_speed", 1.8);
    toggle_cooldown_sec_ = this->declare_parameter<double>("toggle_cooldown_sec", 0.5);

    lookahead_distance_ = base_lookahead_;

    // -- Drivetrain: m/s <-> ERPM conversion (see race_car_parameters.txt) ----
    // VESC ERPM is electrical RPM = mechanical RPM x pole pairs (poles / 2).
    const double motor_poles = this->declare_parameter<double>("motor_poles", 14.0);
    const double gear_ratio = this->declare_parameter<double>("gear_ratio", 2.769);
    const double wheel_radius = this->declare_parameter<double>("wheel_radius", 0.055);
    erpm_per_mps_ = 60.0 * (motor_poles / 2.0) * gear_ratio / (2.0 * M_PI * wheel_radius);

    // -- Topics ---------------------------------------------------------------
    const auto path_topic = this->declare_parameter<std::string>("path_topic", "/selected_path");
    const auto odom_topic = this->declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
    const auto servo_topic = this->declare_parameter<std::string>("servo_topic_pub", "/commands/servo/position");
    const auto motor_topic = this->declare_parameter<std::string>("motor_topic_pub", "/commands/motor/speed");
    const auto target_speed_topic = this->declare_parameter<std::string>("target_speed_topic", "/target_speed");
    const auto vesc_state_topic = this->declare_parameter<std::string>("vesc_state_topic", "/sensors/core");
    const auto joy_topic = this->declare_parameter<std::string>("joy_topic", "/joy");
    const auto marker_topic = this->declare_parameter<std::string>(
        "rviz_lookahead_waypoint_topic", "/ego_racecar/lookahead_waypoint");

    RCLCPP_INFO(this->get_logger(),
                "RaceCarController: wheelbase=%.2f m, max steer=%.2f deg, gain=%.2f, "
                "servo=[%.2f..%.2f], Ld=[%.2f..%.2f] m (base %.2f), speed=[%.2f..%.2f] m/s, "
                "1 m/s = %.1f ERPM",
                wheelbase_, max_steer_angle_deg_, steering_gain_,
                servo_min_, servo_max_, min_lookahead_, max_lookahead_, base_lookahead_,
                min_speed_, max_speed_, erpm_per_mps_);

    steering_pub_ = this->create_publisher<std_msgs::msg::Float64>(servo_topic, 10);
    throttle_pub_ = this->create_publisher<std_msgs::msg::Float64>(motor_topic, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        path_topic, 10, std::bind(&RaceCarController::pathCallback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, std::bind(&RaceCarController::poseCallback, this, std::placeholders::_1));
    vesc_state_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
        vesc_state_topic, 10, std::bind(&RaceCarController::vescStateCallback, this, std::placeholders::_1));
    target_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        target_speed_topic, 10, std::bind(&RaceCarController::targetSpeedCallback, this, std::placeholders::_1));
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic, 10, std::bind(&RaceCarController::joyCallback, this, std::placeholders::_1));

    last_autonomous_toggle_time_ = this->get_clock()->now();
}

void RaceCarController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    path_ = msg;
}

void RaceCarController::targetSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // /target_speed is in m/s. Safety ceiling only: never exceed max_speed_.
    desired_speed_ = std::min(msg->data, max_speed_);

    // Speed-adaptive lookahead: Ld grows linearly with the commanded speed,
    // anchored so that Ld == base_lookahead_ at min_speed_.
    const double ld = base_lookahead_ * (desired_speed_ / min_speed_);
    lookahead_distance_ = clamp(ld, min_lookahead_, max_lookahead_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "target speed = %.2f m/s (%.0f ERPM), lookahead = %.2f m",
                         desired_speed_, desired_speed_ * erpm_per_mps_,
                         lookahead_distance_);
}

void RaceCarController::vescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
    // The VESC reports speed in ERPM; keep the internal state in m/s.
    current_speed_ = msg->state.speed / erpm_per_mps_;
}

void RaceCarController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    const auto now = this->get_clock()->now();
    if (!msg->buttons.empty() && msg->buttons[0] == 1 &&
        (now - last_autonomous_toggle_time_).seconds() > toggle_cooldown_sec_)
    {
        autonomous_flag_ = !autonomous_flag_;
        last_autonomous_toggle_time_ = now;
        RCLCPP_INFO(this->get_logger(), "Autonomous mode: %s", autonomous_flag_ ? "ON" : "OFF");
    }

    // While idle, keep streaming neutral commands so the car stays stopped.
    if (!autonomous_flag_)
    {
        publishStopCommands();
    }
}

void RaceCarController::publishStopCommands()
{
    std_msgs::msg::Float64 steer_msg;
    steer_msg.data = servo_center_;
    steering_pub_->publish(steer_msg);

    std_msgs::msg::Float64 speed_msg;
    speed_msg.data = kStopSpeedErpm;
    throttle_pub_->publish(speed_msg);
}

size_t RaceCarController::findClosestPointAhead(const geometry_msgs::msg::Point &position,
                                                double heading_x, double heading_y) const
{
    size_t closest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();

    for (size_t i = 0; i < path_->poses.size(); ++i)
    {
        const auto &pt = path_->poses[i].pose.position;
        const double vx = pt.x - position.x;
        const double vy = pt.y - position.y;

        // Skip points behind the car.
        if (heading_x * vx + heading_y * vy < 0.0)
            continue;

        const double dist_sq = vx * vx + vy * vy;
        if (dist_sq < min_dist_sq)
        {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }
    return closest_idx;
}

size_t RaceCarController::findLookaheadIndex(size_t start_idx) const
{
    const size_t n = path_->poses.size();
    size_t idx = start_idx;
    double acc_dist = 0.0;

    // Bounded by n steps so degenerate paths (all points identical or total
    // length shorter than Ld on an open path) cannot spin forever.
    for (size_t steps = 0; acc_dist < lookahead_distance_ && steps < n; ++steps)
    {
        const size_t next_idx = (idx + 1) % n;
        const auto &p1 = path_->poses[idx].pose.position;
        const auto &p2 = path_->poses[next_idx].pose.position;
        acc_dist += std::hypot(p2.x - p1.x, p2.y - p1.y);
        idx = next_idx;
    }
    return idx;
}

double RaceCarController::computeServoCommand(double alpha, double lookahead_dist) const
{
    // Geometric pure pursuit steering angle.
    const double delta_deg =
        std::atan2(2.0 * wheelbase_ * std::sin(alpha), lookahead_dist) * kRadToDeg;

    // Proportional gain, normalized by the physical steering limit.
    const double u = clamp(steering_gain_ * delta_deg / max_steer_angle_deg_, -1.0, 1.0);

    return servo_center_ + u * (servo_max_ - servo_min_) / 2.0;
}

void RaceCarController::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!autonomous_flag_ || !path_ || path_->poses.empty())
        return;

    const double yaw = tf2::getYaw(msg->pose.pose.orientation);
    const double hx = std::cos(yaw);
    const double hy = std::sin(yaw);
    const auto &curr_pt = msg->pose.pose.position;

    const size_t closest_idx = findClosestPointAhead(curr_pt, hx, hy);
    const size_t lookahead_idx = findLookaheadIndex(closest_idx);
    const auto &lookahead_pt = path_->poses[lookahead_idx].pose.position;

    publishLookaheadMarker(lookahead_pt);

    const double dx = lookahead_pt.x - curr_pt.x;
    const double dy = lookahead_pt.y - curr_pt.y;
    double alpha = std::atan2(dy, dx) - yaw;
    while (alpha > M_PI)
        alpha -= 2.0 * M_PI;
    while (alpha < -M_PI)
        alpha += 2.0 * M_PI;

    std_msgs::msg::Float64 steer_msg;
    steer_msg.data = computeServoCommand(alpha, std::hypot(dx, dy));
    steering_pub_->publish(steer_msg);

    std_msgs::msg::Float64 speed_msg;
    // m/s -> ERPM: the VESC driver expects ERPM on the motor speed topic.
    speed_msg.data = desired_speed_ * erpm_per_mps_;
    throttle_pub_->publish(speed_msg);
}

void RaceCarController::publishLookaheadMarker(const geometry_msgs::msg::Point &point)
{
    // Marker serialization is wasted work when RViz is not connected
    // (the normal case on the car), so skip it entirely.
    if (marker_pub_->get_subscription_count() == 0)
        return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "lookahead_target";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker_pub_->publish(marker);
}

} // namespace racecar

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<racecar::RaceCarController>());
    rclcpp::shutdown();
    return 0;
}
