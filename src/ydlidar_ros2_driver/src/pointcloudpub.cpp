#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <laser_geometry/laser_geometry.hpp>

class LaserScanToPointCloudNode : public rclcpp::Node {
public:
  LaserScanToPointCloudNode()
  : Node("laser_scan_to_pointcloud_node")
  {
    // Subscriber to LaserScan
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&LaserScanToPointCloudNode::scanCallback, this, std::placeholders::_1));

    // Publisher for PointCloud2
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

    RCLCPP_INFO(this->get_logger(), "LaserScan to PointCloud2 node has been started.");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    try {
      projector_.projectLaser(*scan_msg, cloud_msg);
      cloud_msg.header = scan_msg->header;
      cloud_pub_->publish(cloud_msg);
    } catch (std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Laser projection failed: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  laser_geometry::LaserProjection projector_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanToPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}
