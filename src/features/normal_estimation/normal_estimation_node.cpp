#include "pcl_ros2/features/normal_estimation_node.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <rclcpp/logging.hpp>
// #include <pcl/filters/passthrough.h>



namespace pcl_ros2
{
  NormalEstimationFeatureComponent::NormalEstimationFeatureComponent(const rclcpp::NodeOptions & options)
  : FilterNode("normal_estimation_node", options)
  {
    // parameter_listener_ = std::make_shared<normal_estimation::ParamListener>(this->get_node_parameters_interface());
  }

  void NormalEstimationFeatureComponent::Filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "msg received");
    // auto params = parameter_listener_->get_params();
    pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);;
    pcl_conversions::toPCL(*msg, *pcl_pc2);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*pcl_pc2, *cloud);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.1);
    ne.compute(*cloud_normals);
    for(size_t i = 0; i < 3; ++i) {
      RCLCPP_INFO_STREAM(this->get_logger(), "    normal: " << cloud_normals->points[i].normal_x << " " << cloud_normals->points[i].normal_y << " " << cloud_normals->points[i].normal_z);
    }
    
    // append normal estimation to pcl_pc2
    // pcl::PCLPointCloud2::Ptr cloud_normals_pc2(new pcl::PCLPointCloud2);
    // pcl::toPCLPointCloud2(*cloud_normals, *cloud_normals_pc2);
    // pcl::concatenateFields(*pcl_pc2, *cloud_normals_pc2, *pcl_pc2);

    sensor_msgs::msg::PointCloud2::SharedPtr msg_filtered(new sensor_msgs::msg::PointCloud2);
    pcl_conversions::fromPCL(*pcl_pc2, *msg_filtered);

    pub_->publish(*msg_filtered);
  }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros2::NormalEstimationFeatureComponent)
