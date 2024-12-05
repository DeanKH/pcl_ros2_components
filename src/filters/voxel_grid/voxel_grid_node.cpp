#include "pcl_ros2/filters/voxel_grid_node.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include "rclcpp_components/register_node_macro.hpp"

namespace pcl_ros2
{

VoxelGridFilterComponent::VoxelGridFilterComponent(const rclcpp::NodeOptions & options)
: FilterNode("voxel_grid_node", options)
{
  parameter_listener_ = std::make_shared<voxel_grid::ParamListener>(this->get_node_parameters_interface());
}

void VoxelGridFilterComponent::Filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto params = parameter_listener_->get_params();
  float leaf_x = static_cast<float>(params.leaf_size.x);
  float leaf_y = static_cast<float>(params.leaf_size.y);
  float leaf_z = static_cast<float>(params.leaf_size.z);
  pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);;
  pcl_conversions::toPCL(*msg, *pcl_pc2);

  pcl::VoxelGrid<pcl::PCLPointCloud2> vgf;
  vgf.setInputCloud(pcl_pc2);
  vgf.setLeafSize(leaf_x, leaf_y, leaf_z);
  pcl::PCLPointCloud2::Ptr pcl_pc2_filtered(new pcl::PCLPointCloud2);
  vgf.filter(*pcl_pc2_filtered);

  sensor_msgs::msg::PointCloud2::SharedPtr msg_filtered(new sensor_msgs::msg::PointCloud2);
  pcl_conversions::fromPCL(*pcl_pc2_filtered, *msg_filtered);

  pub_->publish(*msg_filtered);
}
}  // namespace pcl_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros2::VoxelGridFilterComponent)
