// Copyright (c) 2022 Jonas Mahler
// This file is part of pcl_example.


#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>

#include "pointcloud_filter/pointcloud_filter_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;
    
Pointcloud_filter::Pointcloud_filter(const rclcpp::NodeOptions& options) : Node("pointcloud_filter",options) 
{
      
  declare_parameter<std::string>("topic_pointcloud_in","bf_lidar/point_cloud_out");
  declare_parameter<std::string>("topic_pointcloud_out", "bf_lidar/point_cloud_pointcloud_filter");
  
  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out = get_parameter("topic_pointcloud_out").as_string();
  
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out,2);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  param_topic_pointcloud_in, 10, std::bind(&Pointcloud_filter::topic_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "\n"
  "Node:       pointcloud_filter\n"
  "Subscribes: Pointcloud2 message: %s\n"
  "Publishes:  Pointcloud2 message: %s \n"
  "Details:    No filter applied in this example.\n"
  "Running...", param_topic_pointcloud_in.c_str(),param_topic_pointcloud_out.c_str());

}

void Pointcloud_filter::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  unsigned int num_points = msg->width;
  RCLCPP_INFO(this->get_logger(), "The number of points in the input pointcloud is %i", num_points);
    

  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // ROS2 Pointcloud2 to PCL Pointcloud2
  
  pcl_conversions::toPCL(*msg,*cloud);    
                       
  // Insert your pcl object here
  // -----------------------------------

  cloud_filtered = cloud;

  //------------------------------------

  // PCL message to ROS2 message 

  sensor_msgs::msg::PointCloud2 cloud_out;

  pcl_conversions::fromPCL(*cloud_filtered,cloud_out);  

  unsigned int num_points_out = cloud_out.width;
  RCLCPP_INFO(this->get_logger(), "The number of points in the Pointcloud_filter_out pointcloud is %i", num_points_out);

  cloud_out.header.frame_id = msg->header.frame_id;
  cloud_out.header.stamp = msg->header.stamp;

  // Publish to ROS2 network
  publisher_->publish(cloud_out);
}
