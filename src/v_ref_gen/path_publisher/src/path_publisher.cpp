#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <fstream>
#include <sstream>
#include <string>

class VrefGen : public rclcpp::Node
{
public:
    VrefGen()
        : Node("v_ref_gen")
    {
        // QoS profile
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        this->declare_parameter<std::string>("path_file", "/smooth_loop_path");
        this->declare_parameter<bool>("speed_from_path", false);
        // Get parameters
        std::string path_file = this->get_parameter("path_file").as_string();
        speed_from_path = this->get_parameter("speed_from_path").as_bool();

        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", qos);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&VrefGen::timerCallback, this));

        path_msg_.header.frame_id = "odom";

        loadPathFromCSV(path_file);
    }

private:
    void loadPathFromCSV(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open CSV: %s", filename.c_str());
            return;
        }

        std::string line;
        std::getline(file, line); // read header

        std::stringstream header_stream(line);
        std::string cell;

        int x_index = -1;
        int y_index = -1;
        int speed_index = -1;
        int col = 0;

        // ---- Find column indices ----
        while (std::getline(header_stream, cell, ','))
        {
            if (cell == "x")
                x_index = col;
            if (cell == "y")
                y_index = col;
            if (cell == "desired_speed")
                speed_index = col;
            col++;
        }

        if (x_index < 0 || y_index < 0 || speed_index < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CSV must have x, y, desired_speed columns");
            return;
        }

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::vector<std::string> row;

            while (std::getline(ss, cell, ','))
                row.push_back(cell);

            if (row.size() <= std::max({x_index, y_index, speed_index}))
                continue;

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "odom";

            pose.pose.position.x = std::stod(row[x_index]);
            pose.pose.position.y = std::stod(row[y_index]);
            pose.pose.position.z = 0.0;

            if (speed_from_path)
            {
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = std::stod(row[speed_index]);
            }
            else
            {
                // Calculate orientation based on previous point
                if (!path_msg_.poses.empty())
                {
                    auto &prev_pose = path_msg_.poses.back();
                    double dx = pose.pose.position.x - prev_pose.pose.position.x;
                    double dy = pose.pose.position.y - prev_pose.pose.position.y;
                    double yaw = std::atan2(dy, dx);

                    // Update previous pose orientation
                    prev_pose.pose.orientation.x = 0.0;
                    prev_pose.pose.orientation.y = 0.0;
                    prev_pose.pose.orientation.z = std::sin(yaw / 2.0);
                    prev_pose.pose.orientation.w = std::cos(yaw / 2.0);
                }

                // Default orientation for current point (will be updated when next point is loaded)
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
            }

            path_msg_.poses.push_back(pose);
        }

        // Update the last point's orientation to match the second-to-last
        if (!speed_from_path && path_msg_.poses.size() >= 2)
        {
            path_msg_.poses.back().pose.orientation =
                path_msg_.poses[path_msg_.poses.size() - 2].pose.orientation;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Loaded %zu waypoints from %s.",
            path_msg_.poses.size(), filename.c_str());
    }

    void timerCallback()
    {
        auto now = this->get_clock()->now();

        path_msg_.header.stamp = now;

        for (auto &pose : path_msg_.poses)
            pose.header.stamp = now;

        publisher_->publish(path_msg_);
    }

    nav_msgs::msg::Path path_msg_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool speed_from_path;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VrefGen>());
    rclcpp::shutdown();
    return 0;
}
