generate_parameter_library(
  pass_through_filter_parameters
  parameters.yaml
)
add_library(pass_through_filter_component SHARED pass_through_node.cpp ../filter.cpp)

rclcpp_components_register_node(pass_through_filter_component
  PLUGIN "pcl_ros2::PassThroughFilterComponent"
  EXECUTABLE pass_through_filter_node)
target_link_libraries(pass_through_filter_component ${PCL_LIBRARIES} pass_through_filter_parameters)
ament_target_dependencies(pass_through_filter_component ${dependencies})

install(TARGETS
  pass_through_filter_component
  pass_through_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
