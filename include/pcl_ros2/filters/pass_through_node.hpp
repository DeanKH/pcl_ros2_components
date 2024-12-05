#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pass_through_filter_parameters.hpp"
#include "pcl_ros2/filters/filter.hpp"

namespace pcl_ros2
{
class PassThroughFilterComponent: public FilterNode
{
public:
  explicit PassThroughFilterComponent(const rclcpp::NodeOptions & options);

private:
  void Filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg) override;
  std::shared_ptr<pass_through::ParamListener> parameter_listener_;
};
}
