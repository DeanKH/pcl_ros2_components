#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "voxel_grid_filter_parameters.hpp"
#include "pcl_ros2/filters/filter.hpp"

namespace pcl_ros2
{
class VoxelGridFilterComponent: public FilterNode
{
public:
  explicit VoxelGridFilterComponent(const rclcpp::NodeOptions & options);

private:
  void Filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg) override;
  std::shared_ptr<voxel_grid::ParamListener> parameter_listener_;
};
}
