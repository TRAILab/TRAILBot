// Copyright (c) 2022 Jonas Mahler
// This file is part of pcl_example.


#ifndef POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_
#define POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"  
#include "sensor_msgs/msg/point_cloud2.hpp"

/**
 * @class pointcloud_filter::Pointcloud_filter
 * @brief Receives Pointcloud2 message from lidar sensor and filter its points with an optional pcl filter.
 * 
 */
class Pointcloud_filter : public rclcpp::Node
{
  public:
    
    /**
     * @brief A constructor for Pointcloud_filter::Pointcloud_filter class
     * @param options Additional options to control creation of the node.
     */
    explicit Pointcloud_filter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    /**
     * @brief A destructor for Pointcloud_filter::Pointcloud_filter class
     */
    ~Pointcloud_filter() {};

  protected:
    /**
     * @brief Use a no filter of pcl library
     * @param msg Pointcloud2 message receveived from the ros2 node
     * @return -
     * @details Omit pointcloud filtering in this example
     */
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

 
    // ROS2 subscriber and related topic name
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::string param_topic_pointcloud_in;
    
    // ROS2 publisher and related topic name 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string param_topic_pointcloud_out;
    
};

#endif //POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_
