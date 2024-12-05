set(filter_name "statistical_outlier_removal")
generate_parameter_library(
  ${filter_name}_filter_parameters
  parameters.yaml
)
add_library(${filter_name}_filter_component SHARED ${filter_name}_node.cpp ../filter.cpp)

rclcpp_components_register_node(${filter_name}_filter_component
  PLUGIN "pcl_ros2::StatisticalOutlierRemovalFilterComponent"
  EXECUTABLE ${filter_name}_filter_node)
target_link_libraries(${filter_name}_filter_component ${PCL_LIBRARIES} ${filter_name}_filter_parameters)
ament_target_dependencies(${filter_name}_filter_component ${dependencies})

install(TARGETS
  statistical_outlier_removal_filter_component
  statistical_outlier_removal_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
