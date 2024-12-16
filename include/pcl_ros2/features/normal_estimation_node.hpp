#pragma once
#include "pcl_ros2/filters/filter.hpp"
#include <pcl/features/normal_3d.h>

namespace pcl_ros2
{
class NormalEstimationFeatureComponent: public FilterNode
{
public:
  explicit NormalEstimationFeatureComponent(const rclcpp::NodeOptions & options);
  void Filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg) override;
};
}