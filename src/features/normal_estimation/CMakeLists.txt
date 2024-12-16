# generate_parameter_library(
#   pass_through_filter_parameters
#   parameters.yaml
# )
add_library(normal_estimation_feature_component SHARED normal_estimation_node.cpp ../../filters/filter.cpp)

rclcpp_components_register_node(normal_estimation_feature_component
  PLUGIN "pcl_ros2::NormalEstimationFeatureComponent"
  EXECUTABLE normal_estimation_feature_node)
target_link_libraries(normal_estimation_feature_component ${PCL_LIBRARIES})
ament_target_dependencies(normal_estimation_feature_component ${dependencies})

install(TARGETS
normal_estimation_feature_component
normal_estimation_feature_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
