#include <string>
#include "pcl_ros2/filters/filter.hpp"

namespace pcl_ros2
{
FilterNode::FilterNode(const std::string& node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);

  // Create a subscriber for the input point cloud
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", 10, std::bind(&FilterNode::Filter, this, std::placeholders::_1));
}
}