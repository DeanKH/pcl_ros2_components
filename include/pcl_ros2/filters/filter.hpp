#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


namespace pcl_ros2
{
class FilterNode: public rclcpp::Node
{
public:
  explicit FilterNode(const std::string& node_name, const rclcpp::NodeOptions & options);
protected:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
private:
  virtual void Filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg) = 0;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};
}