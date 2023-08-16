// Copyright (c) 2022 Jonas Mahler
// This file is part of pcl_example.


#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <vector> 

#include "pointcloud_filter/pointcloud_filter_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using std::placeholders::_1;
    
Pointcloud_filter::Pointcloud_filter(const rclcpp::NodeOptions& options) : Node("pointcloud_filter",options) 
{
      
  // declare_parameter<std::string>("topic_pointcloud_in","bf_lidar/point_cloud_out");
  declare_parameter<std::string>("topic_pointcloud_in","/velodyne_points"); //current msg topic to sub to
  declare_parameter<std::string>("topic_pointcloud_out", "/pointcloud_filtered");
  
  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out = get_parameter("topic_pointcloud_out").as_string();
  
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out,2);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_in, 10, std::bind(&Pointcloud_filter::topic_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "\n"
  "Node:       pointcloud_filter\n"
  "Subscribes: Pointcloud2 message: %s\n"
  "Publishes:  Pointcloud2 message: %s \n"
  "Details:    No filter applied in this example.\n"
  "Running...", param_topic_pointcloud_in.c_str(),param_topic_pointcloud_out.c_str());

}

void Pointcloud_filter::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  unsigned int num_points = msg->width; //takes the width of the pointcloud2 msg (number of points or elements in data structure)
  RCLCPP_INFO(this->get_logger(), "The number of points in the input pointcloud is %i", num_points);

  auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(num_points, 1);
  auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
  for (size_t idx = 0; iter_x != iter_x.end();
       ++idx, ++iter_x, ++iter_y, ++iter_z) {
    // cartesian coordinates
    point_cloud->at(idx).x = *iter_x;
    point_cloud->at(idx).y = *iter_y;
    point_cloud->at(idx).z = *iter_z;
  }
  // ROS2 Pointcloud2 to PCL Pointcloud2
  
  //pcl_conversions::toPCL(*msg,*cloud);    //copying our existing message into cloud (pcl object)
                       
  // Insert your pcl object here
  // -----------------------------------

  //cloud_filtered = cloud;
  std::vector<int> indices;
  indices.reserve(point_cloud->size()); //allocate memory for the vector based on the size of the pointcloud

  for (size_t i = 0; i < point_cloud->size(); ++i) {
    if (point_cloud->at(i).z > -2.5)//&& (*point_cloud)[i].phi < -M_PI)
      indices.emplace_back(i);
  }
  *cloud_filtered = pcl::PointCloud<pcl::PointXYZ>(*point_cloud, indices);

  //------------------------------------

  // PCL message to ROS2 message 

  sensor_msgs::msg::PointCloud2 cloud_out;
  pcl::toROSMsg(*cloud_filtered, cloud_out); //converts the pcl object to a ROS2 message (sensor_msgs::msg::PointCloud2

  // pcl_conversions::fromPCL(*cloud_filtered,cloud_out);  

  // unsigned int num_points_out = cloud_out.width;
  // RCLCPP_INFO(this->get_logger(), "The number of points in the Pointcloud_filter_out pointcloud is %i", num_points_out);

  cloud_out.header.frame_id = msg->header.frame_id;
  cloud_out.header.stamp = msg->header.stamp;

  // Publish to ROS2 network
  publisher_->publish(cloud_out);
}
