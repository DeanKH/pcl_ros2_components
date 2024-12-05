#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "statistical_outlier_removal_filter_parameters.hpp"
#include "pcl_ros2/filters/filter.hpp"

namespace pcl_ros2
{
class StatisticalOutlierRemovalFilterComponent: public FilterNode
{
public:
  explicit StatisticalOutlierRemovalFilterComponent(const rclcpp::NodeOptions & options);

private:
  void Filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg) override;
  std::shared_ptr<statistical_outlier_removal::ParamListener> parameter_listener_;
};
}
