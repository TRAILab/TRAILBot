// Copyright (c) 2022 Jonas Mahler
// This file is part of pcl_example.

#include <pointcloud_filter/pointcloud_filter_node.hpp>


int main(int argc, char * argv[])
{
  const rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pointcloud_filter>());
  rclcpp::shutdown();
  return 0;
}
