
generate_parameter_library(
  voxel_grid_filter_parameters
  parameters.yaml
)
add_library(voxel_grid_filter_component SHARED voxel_grid_node.cpp ../filter.cpp)

rclcpp_components_register_node(voxel_grid_filter_component
  PLUGIN "pcl_ros2::VoxelGridFilterComponent"
  EXECUTABLE voxel_grid_filter_node)
target_link_libraries(voxel_grid_filter_component ${PCL_LIBRARIES} voxel_grid_filter_parameters)
ament_target_dependencies(voxel_grid_filter_component ${dependencies})

install(TARGETS
  voxel_grid_filter_component
  voxel_grid_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
