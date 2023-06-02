cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_laserscan_converter)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rclcpp_components_register_node(laserscan_to_pointcloud
  PLUGIN "pointcloud_to_laserscan::LaserScanToPointCloudNode"
  EXECUTABLE laserscan_to_pointcloud_node)

ament_auto_add_library(pointcloud_to_laserscan_converter SHARED
  src/pointcloud_to_laserscan_node.cpp)

rclcpp_components_register_node(pointcloud_to_laserscan
  PLUGIN "pointcloud_to_laserscan_converter::PointCloudToLaserScanNode"
  EXECUTABLE pointcloud_to_laserscan_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
