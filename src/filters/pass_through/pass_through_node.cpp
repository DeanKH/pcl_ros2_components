#include "pcl_ros2/filters/pass_through_node.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include "rclcpp_components/register_node_macro.hpp"

namespace pcl_ros2
{

PassThroughFilterComponent::PassThroughFilterComponent(const rclcpp::NodeOptions & options)
: FilterNode("pass_through_node", options)
{
  parameter_listener_ = std::make_shared<pass_through::ParamListener>(this->get_node_parameters_interface());
}

void PassThroughFilterComponent::Filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto params = parameter_listener_->get_params();
  pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);;
  pcl_conversions::toPCL(*msg, *pcl_pc2);

  pcl::PassThrough<pcl::PCLPointCloud2> filter;
  if(params.field.x.enable) {
    filter.setInputCloud(pcl_pc2);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(params.field.x.min, params.field.x.max);
    // filter.setFilterLimitsNegative(params.field.x.negative);
    filter.filter(*pcl_pc2);
  }

  if(params.field.y.enable) {
    filter.setInputCloud(pcl_pc2);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(params.field.y.min, params.field.y.max);
    // filter.setFilterLimitsNegative(params.field.y.negative);
    filter.filter(*pcl_pc2);
  }

  if(params.field.z.enable) {
    filter.setInputCloud(pcl_pc2);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(params.field.z.min, params.field.z.max);
    // filter.setFilterLimitsNegative(params.field.z.negative);
    filter.filter(*pcl_pc2);
  }


  sensor_msgs::msg::PointCloud2::SharedPtr msg_filtered(new sensor_msgs::msg::PointCloud2);
  pcl_conversions::fromPCL(*pcl_pc2, *msg_filtered);

  pub_->publish(*msg_filtered);
}
}  // namespace pcl_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros2::PassThroughFilterComponent)
