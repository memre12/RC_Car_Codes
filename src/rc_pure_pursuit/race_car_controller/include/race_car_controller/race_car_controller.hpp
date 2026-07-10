#ifndef RACE_CAR_CONTROLLER_HPP_
#define RACE_CAR_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // foxy
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // humble

#include <cmath>
#include <string>

namespace racecar
{

/**
 * Pure pursuit path-tracking controller for the RC car.
 *
 * Subscribes to a path (from the lattice planner or the centerline
 * publisher) and localized odometry, and publishes VESC commands:
 *   - servo position in [0, 1]  (0.5 = straight)
 *   - motor speed in ERPM
 *
 * All speeds inside the node (and on /target_speed) are in m/s; the
 * conversion to ERPM happens only at the VESC boundary:
 *
 *   ERPM = v * 60 * (motor_poles / 2) * gear_ratio / (2 * pi * wheel_radius)
 *
 * (VESC ERPM is electrical RPM = mechanical RPM x pole PAIRS.) With the
 * measured drivetrain (14 poles, 2.769 gear, 0.055 m wheels) this gives
 * 1 m/s = 3365.4 ERPM, i.e. the old 2000 ERPM ~= 0.6 m/s.
 *
 * Steering law (classic pure pursuit + proportional gain):
 *
 *   delta = atan2(2 * L * sin(alpha), Ld)      geometric Ackermann angle
 *   u     = clamp(kp * delta / delta_max, -1, 1)
 *   servo = center + u * (servo_max - servo_min) / 2
 *
 * where L is the wheelbase, alpha the heading error to the lookahead
 * point, Ld the actual lookahead distance, delta_max the physical
 * steering limit of the car and kp the proportional steering gain.
 * kp = 1 is the pure geometric mapping; kp > 1 makes the car turn in
 * more aggressively and saturate earlier.
 *
 * A joystick button toggles autonomous mode; when disabled the node
 * streams neutral servo / zero speed commands.
 */
class RaceCarController : public rclcpp::Node
{
public:
    RaceCarController();

private:
    // -- Callbacks ---------------------------------------------------------
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void vescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
    void targetSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // -- Pure pursuit helpers ----------------------------------------------
    /// Index of the path point closest to the car among points in front of it.
    size_t findClosestPointAhead(const geometry_msgs::msg::Point &position,
                                 double heading_x, double heading_y) const;
    /// Walk forward along the path from start_idx until lookahead_distance_
    /// of arc length is accumulated (wraps around closed paths).
    size_t findLookaheadIndex(size_t start_idx) const;
    /// Map a heading error / lookahead pair to a servo position in [0, 1].
    double computeServoCommand(double alpha, double lookahead_dist) const;

    void publishStopCommands();
    void publishLookaheadMarker(const geometry_msgs::msg::Point &point);

    template <typename T>
    static constexpr T clamp(const T &v, const T &lo, const T &hi)
    {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }

    // -- Publishers / subscribers -------------------------------------------
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_speed_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // -- State ---------------------------------------------------------------
    nav_msgs::msg::Path::SharedPtr path_;
    double desired_speed_ = 0.0;   // m/s, commanded by /target_speed
    double current_speed_ = 0.0;   // m/s, converted from the VESC ERPM feedback
    double lookahead_distance_;    // meters, adapted to speed
    bool autonomous_flag_ = false;
    rclcpp::Time last_autonomous_toggle_time_;

    // -- Vehicle geometry / steering parameters ------------------------------
    double wheelbase_;             // meters (0.28 on the real car)
    double max_steer_angle_deg_;   // physical steering limit (29.85 deg)
    double steering_gain_;         // proportional gain kp on the pure pursuit angle
    double servo_min_;             // servo command at full right/left lock
    double servo_max_;
    double servo_center_;          // servo command for straight driving

    // -- Lookahead / speed parameters ----------------------------------------
    double base_lookahead_;        // Ld at min_speed_ (meters)
    double min_lookahead_;         // lower clamp for Ld (meters)
    double max_lookahead_;         // upper clamp for Ld (meters)
    double min_speed_;             // m/s at which Ld == base_lookahead_
    double max_speed_;             // m/s safety ceiling for motor commands

    // -- Drivetrain (m/s <-> ERPM conversion at the VESC boundary) ------------
    double erpm_per_mps_;          // computed from motor_poles / gear_ratio / wheel_radius

    double toggle_cooldown_sec_;   // debounce for the autonomous toggle button
};

} // namespace racecar

#endif // RACE_CAR_CONTROLLER_HPP_
