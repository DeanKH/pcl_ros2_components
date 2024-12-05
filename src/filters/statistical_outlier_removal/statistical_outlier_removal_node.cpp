#include "pcl_ros2/filters/statistical_outlier_removal_node.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "rclcpp_components/register_node_macro.hpp"

namespace pcl_ros2
{

StatisticalOutlierRemovalFilterComponent::StatisticalOutlierRemovalFilterComponent(const rclcpp::NodeOptions & options)
: FilterNode("statistical_outlier_removal_node", options)
{
  parameter_listener_ = std::make_shared<statistical_outlier_removal::ParamListener>(this->get_node_parameters_interface());
}

void StatisticalOutlierRemovalFilterComponent::Filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto params = parameter_listener_->get_params();
  int mean_k = static_cast<int>(params.mean_k);
  double stddev_mul_thresh = params.std_dev_mul_thresh;
  pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);;
  pcl_conversions::toPCL(*msg, *pcl_pc2);

  pcl::PCLPointCloud2::Ptr pcl_pc2_filtered(new pcl::PCLPointCloud2);
  if(pcl_pc2->data.size() < static_cast<size_t>(mean_k)) {
    RCLCPP_WARN(get_logger(), "Input point cloud size is less than mean_k. Skipping filtering.");
    pcl_pc2_filtered = pcl_pc2;
  }else {
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(pcl_pc2);
    filter.setMeanK(mean_k);
    filter.setStddevMulThresh(stddev_mul_thresh);
    filter.filter(*pcl_pc2_filtered);
  }

  sensor_msgs::msg::PointCloud2::SharedPtr msg_filtered(new sensor_msgs::msg::PointCloud2);
  pcl_conversions::fromPCL(*pcl_pc2_filtered, *msg_filtered);

  pub_->publish(*msg_filtered);
}
}  // namespace pcl_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros2::StatisticalOutlierRemovalFilterComponent)
