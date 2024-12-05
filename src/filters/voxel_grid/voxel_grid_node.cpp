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
  // RCLCPP_INFO_STREAM(this->get_logger(), "receive new data: " << msg->header.stamp.sec << "." << msg->header.stamp.nanosec);
  // Convert the sensor_msgs::msg::PointCloud2 message to a pcl::PointCloud
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> vgf;
  vgf.setInputCloud(cloud);
  vgf.setLeafSize(leaf_x, leaf_y, leaf_z);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  vgf.filter(*cloud_filtered);

  // Convert the pcl::PointCloud back to a sensor_msgs::msg::PointCloud2 message
  pcl::PCLPointCloud2 pcl_pc2_filtered;
  pcl::toPCLPointCloud2(*cloud_filtered, pcl_pc2_filtered);
  sensor_msgs::msg::PointCloud2::SharedPtr msg_filtered(new sensor_msgs::msg::PointCloud2);
  pcl_conversions::fromPCL(pcl_pc2_filtered, *msg_filtered);

  // Publish the filtered point cloud
  pub_->publish(*msg_filtered);
}
}  // namespace pcl_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros2::VoxelGridFilterComponent)
