# pcl_ros2

This package is inspired by [perception_pcl](https://github.com/ros-perception/perception_pcl).

The purpose of this package is that make it easy for anyone to implement pcl functions combination by component container.

## example

### VoxelGrid

```shell
ros2 run pcl_ros2 voxel_grid_filter_node --ros-args -r input:=/camera/camera/depth/color/points -r output:=voxelgrid/output
```

### PassThrough

```shell
ros2 run pcl_ros2 pass_through_filter_node --ros-args -r input:=/camera/camera/depth/color/points -r output:=/passthrough/output
```

### StatisticalOutlierRemoval

```shell
ros2 run pcl_ros2 statistical_outlier_removal_filter_node --ros-args -r input:=/camera/camera/depth/color/points -r output:=sor/output
```
